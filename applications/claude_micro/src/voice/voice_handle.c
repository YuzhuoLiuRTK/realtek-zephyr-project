/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Voice data flow for claude_micro.
 *
 * Raw PCM path (no encoding):
 *   i2s_read() → app_usb_audio_send() → USB UAC ring buffer → host
 *
 * The rx thread is the only producer of the UAC ring buffer and the USB SOF
 * callback the only consumer, so the PCM is copied straight out of the I2S
 * slab block into the ring buffer and the block is freed right away.
 *
 * Thread lifecycle
 * ────────────────
 * The rx thread is created once and loops forever.  Each "recording session"
 * is gated by two semaphores:
 *
 *   sem_start  – posted by voice_handle_mic_key_pressed()  → thread wakes
 *   sem_stopped – posted by the thread when it exits a session cleanly
 *              → voice_handle_mic_key_released() can wait for a clean stop
 *
 * The thread checks the atomic flag `voice_rx_running` each iteration.
 * When cleared it drains the current i2s_read(), frees the slab, and posts
 * sem_stopped before blocking again on sem_start.  This guarantees the slab
 * blocks are never leaked between sessions.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <config.h>
#include <led/led_driver.h>
#include <usb/usb_app.h>
#include <voice/voice_driver.h>
#include <voice/voice_handle.h>

LOG_MODULE_DECLARE(app, CONFIG_APP_LOG_LEVEL);

/* ── Thread ───────────────────────────────────────────────────────────────── */
static struct k_thread voice_rx_data;
K_THREAD_STACK_DEFINE(voice_rx_stack, 1024);

/* ── Session control ──────────────────────────────────────────────────────── */
static K_SEM_DEFINE(sem_start,   0, 1);   /* pressed  → thread wake  */
static K_SEM_DEFINE(sem_stopped, 0, 1);   /* thread   → clean exit   */
static atomic_t voice_rx_running = ATOMIC_INIT(0);

/* Bytes forwarded to USB in the current session (for logging only) */
static uint32_t voice_data_total_cnt;

/* ── RX thread ────────────────────────────────────────────────────────────── */
static void voice_rx_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_INF("voice rx thread started");

	while (true) {
		/* ── Wait for a new recording session ─────────────────────── */
		k_sem_take(&sem_start, K_FOREVER);
		LOG_INF("voice rx: session begin");

		/* ── Recording loop ───────────────────────────────────────── */
		while (atomic_get(&voice_rx_running)) {
			void  *mem_block;
			size_t rx_size;

			int ret = i2s_read(dev_i2s, &mem_block, &rx_size);
			if (ret) {
				if (!atomic_get(&voice_rx_running)) {
					/* Normal: stop was requested, timeout/-EIO expected */
					break;
				}
				if (ret == -EAGAIN) {
					/* Transient underrun — keep trying */
					continue;
				}
				LOG_ERR("i2s_read failed: %d", ret);
				break;
			}

			uint32_t len = MIN(rx_size, (size_t)VOICE_BLOCK_SIZE);

			app_usb_audio_send(mem_block, (int)len);
			voice_data_total_cnt += len;

			/* Free right away so DMA can reuse the slab block */
			const struct i2s_config *rx_cfg =
				i2s_config_get(dev_i2s, I2S_DIR_RX);
			k_mem_slab_free(rx_cfg->mem_slab, mem_block);
		}

		/* ── Session ended — signal caller ───────────────────────── */
		LOG_INF("voice rx: session end (total=%u B)", voice_data_total_cnt);
		k_sem_give(&sem_stopped);
	}
}

/* ── Public API ───────────────────────────────────────────────────────────── */
static bool rx_thread_created;

bool voice_handle_mic_key_pressed(void)
{
	if (voice_driver_global_data.is_voice_driver_working) {
		LOG_WRN("mic already active, ignoring press");
		return false;
	}

	LOG_INF("mic start");
	voice_data_total_cnt = 0U;

	voice_driver_init();

	/* Arm the running flag before waking the thread so it never misses it */
	atomic_set(&voice_rx_running, 1);

	if (!rx_thread_created) {
		k_tid_t tid = k_thread_create(
			&voice_rx_data, voice_rx_stack,
			K_THREAD_STACK_SIZEOF(voice_rx_stack),
			voice_rx_thread_fn, NULL, NULL, NULL,
			K_PRIO_COOP(10), 0, K_NO_WAIT);
		k_thread_name_set(tid, "voice_rx");
		rx_thread_created = true;
	}

	k_sem_give(&sem_start);
	return true;
}

void voice_handle_mic_key_released(void)
{
	if (!voice_driver_global_data.is_voice_driver_working) {
		LOG_WRN("mic not active, ignoring release");
		return;
	}

	LOG_INF("mic stop");

	/* 1. Signal thread to exit its recording loop */
	atomic_set(&voice_rx_running, 0);

	/* 2. Drop I2S — this causes the pending i2s_read() to return -EIO
	 *    so the thread unblocks immediately instead of waiting for DMA */
	voice_driver_deinit();

	/* 3. Wait for the thread to finish the current session cleanly.
	 *    This ensures the slab is fully freed before the next init. */
	int sem_ret = k_sem_take(&sem_stopped, K_MSEC(500));
	if (sem_ret != 0) {
		LOG_ERR("voice rx thread did not stop in 500 ms (ret=%d)", sem_ret);
	} else {
		LOG_INF("voice rx: thread stopped cleanly");
	}
}

void voice_handle_stop_mic(void)
{
	voice_handle_mic_key_released();
	led_mic_blink_stop();
}
