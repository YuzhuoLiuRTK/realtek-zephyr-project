/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 * SPDX-License-Identifier: Apache-2.0
 *
 * USB interface for claude_micro (UAC2 mic + HID, USB device_next).
 *
 * Init order: app_usb_audio_init() and app_usb_hid_init() register the class
 * ops, then app_usbd_init() initialises and enables the device stack.
 */

#ifndef _USB_APP_H_
#define _USB_APP_H_

#include <stdbool.h>
#include <stdint.h>

/* ── UAC2 microphone (usb_audio.c) ──────────────────────────────────────── */
void app_usb_audio_init(void);

/**
 * @brief Queue raw 16-bit mono PCM for the UAC2 IN stream.
 *
 * Single producer only; data that does not fit in the ring buffer is dropped.
 */
void app_usb_audio_send(const uint8_t *pcm_data, int pcm_size);

/* ── HID (usb_hid.c) ────────────────────────────────────────────────────── */
void app_usb_hid_init(void);

/**
 * @brief Report window YES button state to the host.  ISR safe.
 *
 * @param win_mask  bit 0 = window 1, bit 1 = window 2, bit 2 = window 3.
 * @param pressed   true to report @p win_mask, false to report all released.
 */
void app_usb_hid_send_win_key(uint8_t win_mask, bool pressed);

/* ── Device context (usbd_init.c) ───────────────────────────────────────── */
int app_usbd_init(void);

#endif /* _USB_APP_H_ */
