/*
 * Copyright(c) 2026, Realtek Semiconductor Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_audio.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/audio/codec.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* I2S and codec device tree aliases — set in board overlay:
 *   /aliases { test_i2s = &your_i2s; test_codec = &your_codec; };
 */
static const struct device *dev_i2s = DEVICE_DT_GET_OR_NULL(DT_ALIAS(test_i2s));
static const struct device *dev_codec = DEVICE_DT_GET_OR_NULL(DT_ALIAS(test_codec));

/* ------------------------------------------------------------------
 * MIC (I2S → USB) data path
 * ------------------------------------------------------------------
 * USB sends are driven by data_written_cb (transfer completion).
 * The I2S reader thread fills a shared buffer asynchronously.
 *
 * flow:
 *   data_request_cb  → send first frame (I2S data or silence)
 *   data_written_cb  → send next frame  (I2S data or silence) → chain
 *   I2S reader thread → i2s_read loop, fills i2s_buffer only
 */

#define MIC_BLOCK_SIZE	  192
#define NUM_MIC_BLOCKS	  16

/* Memory slab for I2S RX DMA buffers */
K_MEM_SLAB_DEFINE(i2s_rx_slab, MIC_BLOCK_SIZE, NUM_MIC_BLOCKS, 4);

/* Buffer pool for USB MIC transfers */
NET_BUF_POOL_FIXED_DEFINE(mic_pool, NUM_MIC_BLOCKS, MIC_BLOCK_SIZE,
			  4, net_buf_destroy);

/* Shared buffer between I2S reader thread and USB send chain */
static uint8_t i2s_buffer[MIC_BLOCK_SIZE];
static volatile bool buf_has_i2s_data;
static volatile bool tx_active;

static K_SEM_DEFINE(mic_trigger_sem, 0, 1);

static const struct device *mic_dev;

static int i2s_codec_start(void)
{
	struct audio_codec_cfg codec_cfg = { 0 };
	audio_property_value_t val = { .mute = false };

	codec_cfg.mclk_freq = 2500000;
	codec_cfg.dai_type = AUDIO_DAI_TYPE_I2S;
	codec_cfg.dai_cfg.i2s.frame_clk_freq = 48000;
	codec_cfg.dai_cfg.i2s.word_size = 16;
	codec_cfg.dai_cfg.i2s.channels = 2;
	codec_cfg.dai_cfg.i2s.format =
		I2S_FMT_DATA_FORMAT_I2S | I2S_FMT_DATA_ORDER_MSB;
	codec_cfg.dai_cfg.i2s.options =
		I2S_OPT_BIT_CLK_CONT | I2S_OPT_BIT_CLK_MASTER |
		I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_PINGPONG;
	codec_cfg.dai_cfg.i2s.block_size = MIC_BLOCK_SIZE;
	codec_cfg.dai_cfg.i2s.mem_slab = &i2s_rx_slab;
	codec_cfg.dai_cfg.i2s.timeout = SYS_FOREVER_MS;

	if (audio_codec_configure(dev_codec, &codec_cfg)) {
		LOG_ERR("codec configure failed");
		return -EIO;
	}
	audio_codec_set_property(dev_codec, AUDIO_PROPERTY_OUTPUT_MUTE,
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
	LOG_INF("I2S + codec stopped");
}

/* ------------------------------------------------------------------
 * USB send chain: data_written_cb drives the next send.
 * Called from workqueue context after each XFER_COMPL.
 * ------------------------------------------------------------------ */

static void send_next_frame(const struct device *dev)
{
	struct net_buf *tx_buf = net_buf_alloc(&mic_pool, K_NO_WAIT);
	if (tx_buf == NULL) {
		LOG_ERR("mic buffer pool exhausted");
		return;
	}

	if (buf_has_i2s_data) {
		buf_has_i2s_data = false;
		memcpy(net_buf_add(tx_buf, MIC_BLOCK_SIZE),
		       i2s_buffer, MIC_BLOCK_SIZE);
	} else {
		/* No I2S data yet — send silence */
		memset(net_buf_add(tx_buf, MIC_BLOCK_SIZE), 0,
		       MIC_BLOCK_SIZE);
	}

	int ret = usb_audio_send(dev, tx_buf, MIC_BLOCK_SIZE);
	if (ret == -EAGAIN) {
		tx_active = false;
		net_buf_unref(tx_buf);
	} else if (ret) {
		net_buf_unref(tx_buf);
	}
}

static void data_written_cb(const struct device *dev,
			    struct net_buf *buffer, size_t size)
{
	/* Free the buffer from the completed transfer */
	net_buf_unref(buffer);

	/* Chain the next frame */
	if (tx_active) {
		send_next_frame(dev);
	}
}

/* ------------------------------------------------------------------
 * I2S reader thread: fills i2s_buffer, never calls usb_audio_send
 * ------------------------------------------------------------------ */

static void mic_reader_thread(void *arg1, void *arg2, void *arg3)
{
	while (1) {
		k_sem_take(&mic_trigger_sem, K_FOREVER);

		if (i2s_codec_start()) {
			continue;
		}

		while (tx_active) {
			void *block;
			size_t size;
			int ret;

			ret = i2s_read(dev_i2s, &block, &size);
			if (ret) {
				LOG_ERR("i2s_read failed: %d", ret);
				break;
			}

			memcpy(i2s_buffer, block, size);
			buf_has_i2s_data = true;

			const struct i2s_config *cfg =
				i2s_config_get(dev_i2s, I2S_DIR_RX);
			if (cfg != NULL && cfg->mem_slab != NULL) {
				k_mem_slab_free(cfg->mem_slab, block);
			}
		}

		i2s_codec_stop();
	}
}

K_THREAD_DEFINE(mic_tid, 2048, mic_reader_thread, NULL, NULL, NULL,
		K_PRIO_PREEMPT(5), 0, 0);

/* ------------------------------------------------------------------
 * UAC callbacks
 * ------------------------------------------------------------------ */

static void data_request_cb(const struct device *dev)
{
	/* Called when UAC MIC streaming starts (host selected alt=1).
	 * On RTL87X2G this is triggered from USB_DC_INTERFACE.
	 * Start I2S and prime the first USB frame. */
	mic_dev = dev;

	if (dev_i2s == NULL || dev_codec == NULL) {
		LOG_WRN("MIC capture unavailable — no I2S/codec device");
		return;
	}

	tx_active = true;
	buf_has_i2s_data = false;

	/* Wake I2S reader thread */
	k_sem_give(&mic_trigger_sem);

	/* Prime the first USB frame immediately (silence if no I2S data) */
	send_next_frame(dev);
}

static void feature_update(const struct device *dev,
			   const struct usb_audio_fu_evt *evt)
{
	switch (evt->cs) {
	case USB_AUDIO_FU_MUTE_CONTROL:
		LOG_INF("mute %s", *((const uint8_t *)evt->val) ? "on" : "off");
		break;
	case USB_AUDIO_FU_VOLUME_CONTROL:
		LOG_INF("set volume: %d", *((const int16_t *)evt->val));
		break;
	default:
		break;
	}
}

static const struct usb_audio_ops ops = {
	.data_request_cb   = data_request_cb,
	.data_written_cb   = data_written_cb,
	.feature_update_cb = feature_update,
};

/* ------------------------------------------------------------------
 * main
 * ------------------------------------------------------------------ */

int main(void)
{
	int ret;

	LOG_INF("Entered %s", __func__);

	mic_dev = DEVICE_DT_GET_ONE(usb_audio_mic);

	if (!device_is_ready(mic_dev)) {
		LOG_ERR("Device USB Microphone is not ready");
		return 0;
	}

	if (dev_i2s != NULL && device_is_ready(dev_i2s)) {
		LOG_INF("I2S device ready: %s", dev_i2s->name);
	} else {
		LOG_WRN("I2S device not available — MIC capture disabled");
	}

	if (dev_codec != NULL && device_is_ready(dev_codec)) {
		LOG_INF("Codec device ready: %s", dev_codec->name);
	} else {
		LOG_WRN("Codec device not available — MIC capture disabled");
	}

	LOG_INF("Found USB Microphone Device");

	usb_audio_register(mic_dev, &ops);

	ret = usb_enable(NULL);
	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return 0;
	}

	LOG_INF("USB enabled");
	return 0;
}
