/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Button interface for claude_micro (RTL87x2J EVB).
 *
 * button.c owns the GPIOs and calls the key_handle_* callbacks, which are
 * implemented in button_handle.c.  The callbacks run in ISR context.
 */

#ifndef _BUTTON_H_
#define _BUTTON_H_

/**
 * @brief Configure the four button GPIOs and enable their interrupts.
 */
void button_init(void);

/* ── Event callbacks (ISR context, implemented in button_handle.c) ──────── */
void key_handle_mic_press(void);
void key_handle_mic_release(void);
void key_handle_win1_press(void);
void key_handle_win1_release(void);
void key_handle_win2_press(void);
void key_handle_win2_release(void);
void key_handle_win3_press(void);
void key_handle_win3_release(void);

#endif /* _BUTTON_H_ */
