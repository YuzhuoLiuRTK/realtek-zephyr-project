/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 * SPDX-License-Identifier: Apache-2.0
 *
 * All functions block (k_sem_take) and must be called from thread context.
 */

#ifndef _VOICE_HANDLE_H_
#define _VOICE_HANDLE_H_

#include <stdbool.h>

bool voice_handle_mic_key_pressed(void);
void voice_handle_mic_key_released(void);

/* Stop recording and the mic LED blink (used by the 60 s auto-stop). */
void voice_handle_stop_mic(void);

#endif /* _VOICE_HANDLE_H_ */
