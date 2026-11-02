/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _VOICE_DRIVER_H_
#define _VOICE_DRIVER_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>

typedef struct {
	bool is_voice_driver_working;
} T_VOICE_DRIVER_GLOBAL_DATA;

extern T_VOICE_DRIVER_GLOBAL_DATA voice_driver_global_data;

/* I2S device used for mic capture (defined in voice_driver.c) */
extern const struct device *const dev_i2s;

void voice_driver_init(void);
void voice_driver_deinit(void);

#endif /* _VOICE_DRIVER_H_ */
