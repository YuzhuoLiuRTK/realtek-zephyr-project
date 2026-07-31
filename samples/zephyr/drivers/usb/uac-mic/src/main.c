/*
 * Copyright (c) 2026 Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>

#include <sample_usbd.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_uac2.h>
#include <zephyr/drivers/usb/usb_buf.h>	/* UDC_BUF_ALIGN / UDC_BUF_GRANULARITY */

#include <zephyr/drivers/i2s.h>
#include <zephyr/audio/codec.h>

LOG_MODULE_REGISTER(uac_mic, LOG_LEVEL_INF);

/* ------------------------------------------------------------------
 * UAC2 topology (defined in the board overlay, node label "uac2_mic").
 *
 * usbd_uac2_send() and the terminal callbacks address the USB Streaming
 * *output* terminal (in_terminal), i.e. the entity that feeds the IN
 * endpoint (device -> host).
 * ------------------------------------------------------------------ */
#define UAC2_NODE		DT_NODELABEL(uac2_mic)
#define MIC_IN_TERMINAL_ID	UAC2_ENTITY_ID(DT_NODELABEL(in_terminal))

/* Audio format: 48 kHz, 16-bit, 2 channels (matches the RTL87x2g codec cfg). */
#define SAMPLE_FREQUENCY	48000
#define SAMPLE_BIT_WIDTH	16
#define NUMBER_OF_CHANNELS	2
#define BYTES_PER_SAMPLE	DIV_ROUND_UP(SAMPLE_BIT_WIDTH, 8)
#define BYTES_PER_SLOT		(BYTES_PER_SAMPLE * NUMBER_OF_CHANNELS)

/* High-Speed polling period; must match polling-period-us in the overlay.
 * 1000 us = bInterval 4 = 48 samples/packet, ~1000 sends/s (see overlay).
 */
#define HS_POLLING_PERIOD_US	1000
#define HS_MICROFRAMES_PER_POLL	(HS_POLLING_PERIOD_US / 125)

/* Samples the host expects per polling interval: 48 at Full-Speed (1 ms),
 * 6 per microframe * the microframes covered by one High-Speed poll.
 */
#define FS_SAMPLES_PER_SOF	(SAMPLE_FREQUENCY / 1000)
#define HS_SAMPLES_PER_SOF	(SAMPLE_FREQUENCY / 8000 * HS_MICROFRAMES_PER_POLL)
/* Async mic may carry nominal + 1 samples, size the USB buffer for the worst case. */
#define MAX_FRAME_BYTES		((FS_SAMPLES_PER_SOF + 1) * BYTES_PER_SLOT)

/* One I2S DMA block == 1 ms of audio == one Full-Speed frame. */
#define I2S_BLOCK_SIZE		(FS_SAMPLES_PER_SOF * BYTES_PER_SLOT)	/* 192 */
/* Slab must exceed the driver's RX queue depth (CONFIG_I2S_BEE_RX_BLOCK_COUNT
 * = 8) so the DMA can always allocate a fresh block without exhausting the
 * slab and latching I2S_STATE_ERROR.
 */
#define NUM_I2S_BLOCKS		10

/* I2S RX DMA buffers. */
K_MEM_SLAB_DEFINE_STATIC(i2s_rx_slab, I2S_BLOCK_SIZE, NUM_I2S_BLOCKS, 32);

/* USB TX buffers must satisfy the UDC alignment/granularity requirements. */
#define NUM_USB_BUFS		8
K_MEM_SLAB_DEFINE_STATIC(usb_tx_slab,
			 ROUND_UP(MAX_FRAME_BYTES, UDC_BUF_GRANULARITY),
			 NUM_USB_BUFS, UDC_BUF_ALIGN);

/* Decouples the I2S timing (1 ms blocks) from the USB framing
 * (Full-Speed 1 ms / High-Speed 125 us). Single producer (reader thread),
 * single consumer (SOF callback).
 */
#define MIC_RING_SIZE		(I2S_BLOCK_SIZE * NUM_I2S_BLOCKS)
RING_BUF_DECLARE(mic_ring, MIC_RING_SIZE);

/* Asynchronous IN endpoint: the codec clock is free-running (not synchronized
 * to USB SOF), so we send nominal +/- 1 samples per (micro)frame to keep the
 * ring near half full. This lets the host track the device's real sample rate
 * instead of drifting until the buffer over- or under-runs.
 */
#define RING_LOW_WM		(MIC_RING_SIZE / 4)
#define RING_HIGH_WM		(MIC_RING_SIZE - MIC_RING_SIZE / 4)

/* I2S + codec devices, aliased in the board overlay:
 *   /aliases { test-i2s = &i2s0; test-codec = &codec; };
 */
static const struct device *const dev_i2s =
	DEVICE_DT_GET_OR_NULL(DT_ALIAS(test_i2s));
static const struct device *const dev_codec =
	DEVICE_DT_GET_OR_NULL(DT_ALIAS(test_codec));

struct mic_ctx {
	const struct device *uac2_dev;
	bool mic_enabled;
	bool microframes;
};
static struct mic_ctx ctx;

static K_SEM_DEFINE(mic_run_sem, 0, 1);

/* --- diagnostic counters (each written by a single context) --- */
static uint32_t st_blocks;	/* I2S blocks read by reader thread */
static uint32_t st_overflow;	/* ring overflow events (reader) */
static uint32_t st_sof;		/* sof_cb calls while enabled */
static uint32_t st_sent;	/* usbd_uac2_send() == 0 */
static uint32_t st_fail;	/* usbd_uac2_send() != 0 */
static uint32_t st_nobuf;	/* usb_tx_slab exhausted */
static uint32_t st_underrun;	/* ring had < one frame */
static int st_last_err;		/* last non-zero usbd_uac2_send() return */
static uint32_t st_recover;	/* I2S RX error recoveries (DROP+START) */

/* Buffers handed to usbd_uac2_send() but not yet returned via buf_release.
 * The class allows max 2 in flight; track it so sof_cb skips the alloc/copy/
 * send work when the queue is already full instead of bouncing on -EAGAIN.
 */
static atomic_t tx_outstanding;

/* ==================================================================
 * I2S + codec capture pipeline (unchanged data path)
 * ================================================================== */

static int i2s_codec_start(void)
{
	struct audio_codec_cfg codec_cfg = { 0 };
	audio_property_value_t val = { .mute = false };

	codec_cfg.mclk_freq = 2500000;
	codec_cfg.dai_type = AUDIO_DAI_TYPE_I2S;
	codec_cfg.dai_cfg.i2s.frame_clk_freq = SAMPLE_FREQUENCY;
	codec_cfg.dai_cfg.i2s.word_size = SAMPLE_BIT_WIDTH;
	codec_cfg.dai_cfg.i2s.channels = NUMBER_OF_CHANNELS;
	codec_cfg.dai_cfg.i2s.format =
		I2S_FMT_DATA_FORMAT_I2S | I2S_FMT_DATA_ORDER_MSB;
	codec_cfg.dai_cfg.i2s.options =
		I2S_OPT_BIT_CLK_CONT | I2S_OPT_BIT_CLK_CONTROLLER |
		I2S_OPT_FRAME_CLK_CONTROLLER | I2S_OPT_PINGPONG;
	codec_cfg.dai_cfg.i2s.block_size = I2S_BLOCK_SIZE;
	codec_cfg.dai_cfg.i2s.mem_slab = &i2s_rx_slab;
	/* Finite timeout so i2s_read() returns (-EAGAIN when momentarily empty,
	 * -EIO once the bee driver latches I2S_STATE_ERROR) instead of blocking
	 * the reader forever after an overflow. Keep it short for fast recovery.
	 */
	codec_cfg.dai_cfg.i2s.timeout = 20;

	if (audio_codec_configure(dev_codec, &codec_cfg)) {
		LOG_ERR("codec configure failed");
		return -EIO;
	}
	audio_codec_set_property(dev_codec, AUDIO_PROPERTY_OUTPUT_MUTE,
				 AUDIO_CHANNEL_ALL, val);
	val.vol = 0xaf;
	audio_codec_set_property(dev_codec, AUDIO_PROPERTY_OUTPUT_VOLUME,
				 AUDIO_CHANNEL_ALL, val);
	audio_codec_apply_properties(dev_codec);
	audio_codec_start_output(dev_codec);

	if (i2s_configure(dev_i2s, I2S_DIR_RX, &codec_cfg.dai_cfg.i2s)) {
		LOG_ERR("i2s configure failed");
		audio_codec_stop_output(dev_codec);
		return -EIO;
	}

	if (i2s_trigger(dev_i2s, I2S_DIR_RX, I2S_TRIGGER_START)) {
		LOG_ERR("i2s trigger start failed");
		i2s_configure(dev_i2s, I2S_DIR_RX,
			      &(struct i2s_config){ .frame_clk_freq = 0 });
		audio_codec_stop_output(dev_codec);
		return -EIO;
	}

	LOG_INF("I2S + codec started");
	return 0;
}

static void i2s_codec_stop(void)
{
	i2s_trigger(dev_i2s, I2S_DIR_RX, I2S_TRIGGER_DROP);
	i2s_configure(dev_i2s, I2S_DIR_RX,
		      &(struct i2s_config){ .frame_clk_freq = 0 });
	audio_codec_stop_output(dev_codec);
	ring_buf_reset(&mic_ring);
	LOG_INF("I2S + codec stopped");
}

/* Reader thread: pull I2S blocks and push PCM bytes into the ring buffer. */
static void mic_reader_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	while (1) {
		k_sem_take(&mic_run_sem, K_FOREVER);

		/* Guard against a stale give: the host may toggle the interface
		 * off again before we wake up. Only start if still enabled.
		 */
		if (!ctx.mic_enabled) {
			continue;
		}

		if (i2s_codec_start()) {
			continue;
		}

		while (ctx.mic_enabled) {
			void *block;
			size_t size;
			uint32_t put;
			int ret;

			ret = i2s_read(dev_i2s, &block, &size);
			if (ret == -EAGAIN) {
				/* out_queue momentarily empty (RX still
				 * running); keep polling.
				 */
				continue;
			}
			if (ret < 0) {
				/* The bee driver latches I2S_STATE_ERROR after
				 * an out_queue overflow and stops re-arming the
				 * DMA. Re-arm the stream (DROP clears the error
				 * and purges queues; START restarts capture)
				 * instead of tearing the whole pipeline down.
				 */
				st_last_err = ret;
				st_recover++;
				i2s_trigger(dev_i2s, I2S_DIR_RX,
					    I2S_TRIGGER_DROP);
				if (i2s_trigger(dev_i2s, I2S_DIR_RX,
						I2S_TRIGGER_START)) {
					break;
				}
				continue;
			}

			put = ring_buf_put(&mic_ring, block, size);
			if (put < size) {
				st_overflow++;
			}
			st_blocks++;

			k_mem_slab_free(&i2s_rx_slab, block);
		}

		i2s_codec_stop();
	}
}

/*
 * Preemptible, below the cooperative usbd/dwc2 threads (K_PRIO_COOP(8)). The
 * reader's work is tiny (~1000 blocks/sec of ring_buf_put + slab free) and the
 * RX out_queue gives ~8ms of slack, so it does not need to race USB: it runs in
 * the CPU the USB threads leave idle, and they preempt it instantly so the
 * time-critical iso re-arm is never delayed. Making it cooperative and above
 * them (K_PRIO_COOP(7)) stalled the re-arm and overflowed the ring.
 */
K_THREAD_DEFINE(mic_tid, 2048, mic_reader_thread, NULL, NULL, NULL,
		K_PRIO_PREEMPT(1), 0, 0);

/* Diagnostic: dump the pipeline counters once per second. */
static void mic_stats_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	while (1) {
		k_sleep(K_SECONDS(1));
		LOG_INF("en=%d blk=%u ovf=%u rec=%u | sof=%u sent=%u "
			"fail=%u(err %d) nobuf=%u ur=%u fill=%u",
			ctx.mic_enabled, st_blocks, st_overflow, st_recover,
			st_sof, st_sent, st_fail, st_last_err,
			st_nobuf, st_underrun, ring_buf_size_get(&mic_ring));
	}
}

K_THREAD_DEFINE(mic_stats_tid, 1024, mic_stats_thread, NULL, NULL, NULL,
		K_PRIO_PREEMPT(9), 0, 0);

/* ==================================================================
 * UAC2 class callbacks
 * ================================================================== */

static inline int samples_per_sof(void)
{
	return ctx.microframes ? HS_SAMPLES_PER_SOF : FS_SAMPLES_PER_SOF;
}

/* Arm one packet on the IN endpoint if a buffer slot is free. Called twice per
 * SOF so both slots of the class's double buffer stay filled.
 */
static void mic_try_send(void)
{
	uint32_t frame_bytes;
	uint32_t fill;
	uint32_t n;
	int samples;
	void *buf;
	int ret;

	if (!ctx.mic_enabled) {
		return;
	}
	if (atomic_get(&tx_outstanding) >= 2) {
		return;
	}

	/* Adaptive frame size: nudge +/- 1 sample to keep the ring near
	 * half full and compensate for codec-vs-USB clock drift.
	 */
	samples = samples_per_sof();
	fill = ring_buf_size_get(&mic_ring);
	if (fill > RING_HIGH_WM) {
		samples += 1;	/* drain a little faster */
	} else if (fill < RING_LOW_WM) {
		samples -= 1;	/* let the ring refill */
	}
	frame_bytes = samples * BYTES_PER_SLOT;

	if (k_mem_slab_alloc(&usb_tx_slab, &buf, K_NO_WAIT)) {
		st_nobuf++;
		return;
	}

	n = ring_buf_get(&mic_ring, buf, frame_bytes);
	if (n < frame_bytes) {
		/* Underrun: pad with silence so the host clock keeps going. */
		memset((uint8_t *)buf + n, 0, frame_bytes - n);
		st_underrun++;
	}

	ret = usbd_uac2_send(ctx.uac2_dev, MIC_IN_TERMINAL_ID, buf, frame_bytes);
	if (ret) {
		st_fail++;
		st_last_err = ret;
		k_mem_slab_free(&usb_tx_slab, buf);
	} else {
		atomic_inc(&tx_outstanding);
		st_sent++;
	}
}

/* Called every Start of Frame; keep both endpoint buffer slots armed. */
static void mic_sof(const struct device *dev, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	if (!ctx.mic_enabled) {
		return;
	}

	st_sof++;

	/* Top up to the 2-buffer limit; buf_release normally keeps us armed,
	 * this re-primes if both slots drained (e.g. right after enable).
	 */
	mic_try_send();
	mic_try_send();
}

/* Host enabled/disabled the microphone AudioStreaming interface. */
static void mic_terminal_update(const struct device *dev, uint8_t terminal,
				bool enabled, bool microframes, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	if (terminal != MIC_IN_TERMINAL_ID) {
		return;
	}

	ctx.microframes = microframes;

	if (enabled && !ctx.mic_enabled) {
		atomic_set(&tx_outstanding, 0);	/* clear any stale in-flight count */
		ctx.mic_enabled = true;
		k_sem_give(&mic_run_sem);	/* wake the I2S reader thread */
	} else if (!enabled) {
		ctx.mic_enabled = false;	/* reader thread stops the I2S */
	}
}

/* USB stack is done with a buffer previously passed to usbd_uac2_send(). */
static void mic_buf_release(const struct device *dev, uint8_t terminal,
			    void *buf, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(terminal);
	ARG_UNUSED(user_data);

	atomic_dec(&tx_outstanding);
	k_mem_slab_free(&usb_tx_slab, buf);
}

static const struct uac2_ops mic_ops = {
	.sof_cb = mic_sof,
	.terminal_update_cb = mic_terminal_update,
	.buf_release_cb = mic_buf_release,
	/* get_recv_buf/data_recv_cb are only needed for playback (host -> device). */
	/* feedback_cb is not needed: capture (IN) endpoints do not use feedback. */
};

/* ==================================================================
 * main
 * ================================================================== */

int main(void)
{
	struct usbd_context *usbd;
	int ret;

	LOG_INF("Entered %s", __func__);

	ctx.uac2_dev = DEVICE_DT_GET(UAC2_NODE);
	if (!device_is_ready(ctx.uac2_dev)) {
		LOG_ERR("UAC2 device is not ready");
		return 0;
	}
	if (dev_i2s == NULL || !device_is_ready(dev_i2s)) {
		LOG_ERR("I2S device is not ready");
		return 0;
	}
	if (dev_codec == NULL || !device_is_ready(dev_codec)) {
		LOG_ERR("codec device is not ready");
		return 0;
	}

	usbd_uac2_set_ops(ctx.uac2_dev, &mic_ops, &ctx);

	usbd = sample_usbd_init_device(NULL);
	if (usbd == NULL) {
		LOG_ERR("failed to initialize USB device");
		return 0;
	}

	ret = usbd_enable(usbd);
	if (ret) {
		LOG_ERR("failed to enable USB device: %d", ret);
		return 0;
	}

	LOG_INF("UAC2 microphone ready");
	return 0;
}
