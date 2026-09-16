/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: LicenseRef-Realtek-5-Clause
 */

/* Includes ------------------------------------------------------------------*/
#ifndef LIS3DH_H
#define LIS3DH_H

#include <stdint.h>
#include <stdbool.h>
#include "rtl_i2c.h"

/* Defines -------------------------------------------------------------------*/

/* LI23DH Parameters */
#define LIS3DH_CHIP_ID                  (0x33)
#define LIS3DH_I2C_ADDRESS              (0x18)

#define LIS3DH_INT1                     (1)
#define LIS3DH_I2C_MS_BIT               (BIT7)

/* LI23DH Comands */
#define LIS3DH_REG_STATUS_REG_AUX       (0x07)
#define LIS3DH_REG_OUT_ADC1_L           (0x08)
#define LIS3DH_REG_OUT_ADC1_H           (0x09)
#define LIS3DH_REG_OUT_ADC2_L           (0x0A)
#define LIS3DH_REG_OUT_ADC2_H           (0x0B)
#define LIS3DH_REG_OUT_ADC3_L           (0x0C)
#define LIS3DH_REG_OUT_ADC3_H           (0x0D)
#define LIS3DH_REG_WHO_AM_I             (0x0F)
#define LIS3DH_REG_CTRL_REG0            (0x1E)
#define LIS3DH_REG_TEMP_CFG_REG         (0x1F)
#define LIS3DH_REG_CTRL_REG1            (0x20)
#define LIS3DH_REG_CTRL_REG2            (0x21)
#define LIS3DH_REG_CTRL_REG3            (0x22)
#define LIS3DH_REG_CTRL_REG4            (0x23)
#define LIS3DH_REG_CTRL_REG5            (0x24)
#define LIS3DH_REG_CTRL_REG6            (0x25)
#define LIS3DH_REG_REFERENCE            (0x26)
#define LIS3DH_REG_STATUS_REG           (0x27)
#define LIS3DH_REG_OUT_X_L              (0x28)
#define LIS3DH_REG_OUT_X_H              (0x29)
#define LIS3DH_REG_OUT_Y_L              (0x2A)
#define LIS3DH_REG_OUT_Y_H              (0x2B)
#define LIS3DH_REG_OUT_Z_L              (0x2C)
#define LIS3DH_REG_OUT_Z_H              (0x2D)
#define LIS3DH_REG_FIFO_CTRL_REG        (0x2E)
#define LIS3DH_REG_FIFO_SRC_REG         (0x2F)
#define LIS3DH_REG_INT1_CFG             (0x30)
#define LIS3DH_REG_INT1_SRC             (0x31)
#define LIS3DH_REG_INT1_THS             (0x32)
#define LIS3DH_REG_INT1_DURATION        (0x33)
#define LIS3DH_REG_INT2_CFG             (0x34)
#define LIS3DH_REG_INT2_SRC             (0x35)
#define LIS3DH_REG_INT2_THS             (0x36)
#define LIS3DH_REG_INT2_DURATION        (0x37)
#define LIS3DH_REG_CLICK_CFG            (0x38)
#define LIS3DH_REG_CLICK_SRC            (0x39)
#define LIS3DH_REG_CLICK_THS            (0x3A)
#define LIS3DH_REG_TIME_LIMIT           (0x3B)
#define LIS3DH_REG_TIME_LATENCY         (0x3C)
#define LIS3DH_REG_TIME_WINDOW          (0x3D)
#define LIS3DH_REG_ACT_THS              (0x3E)
#define LIS3DH_REG_ACT_DUR              (0x3F)

/* LI23DH FIFO Mode */
#define LIS3DH_FIFO_MODE_BYPASS         (0)
#define LIS3DH_FIFO_MODE_FIFO           (1)
#define LIS3DH_FIFO_MODE_STREAM         (2)
#define LIS3DH_FIFO_MODE_STREAM_FIFO    (3)

/* LI23DH I1 Parameters */
#define LIS3DH_I1_CLICK                 (BIT7)
#define LIS3DH_I1_IA1                   (BIT6)
#define LIS3DH_I1_IA2                   (BIT5)
#define LIS3DH_I1_ZYXDA                 (BIT4)
#define LIS3DH_I1_321DA                 (BIT3)
#define LIS3DH_I1_WTM                   (BIT2)
#define LIS3DH_I1_OVERRUN               (BIT1)

/* LI23DH Data Rate Parameters */
#define LIS3DH_DATA_RATE_5376_HZ        (9)
#define LIS3DH_DATA_RATE_1600_HZ        (8)
#define LIS3DH_DATA_RATE_400_HZ         (7)
#define LIS3DH_DATA_RATE_200_HZ         (6)
#define LIS3DH_DATA_RATE_100_HZ         (5)
#define LIS3DH_DATA_RATE_50_HZ          (4)
#define LIS3DH_DATA_RATE_25_HZ          (3)
#define LIS3DH_DATA_RATE_10_HZ          (2)
#define LIS3DH_DATA_RATE_1_HZ           (1)
#define LIS3DH_DATA_RATE_POWER_DOWN     (0)

/* Functions -----------------------------------------------------------------*/

/**
 * \brief  Initialize LIS3DH (ODR, Range, BDU).
 * \param  i2c_dev: I2C Peripheral instance.
 */
void lis3dh_init(I2C_TypeDef *i2c_dev);

/**
 * \brief  Read Device ID.
 * \return ID (0x33).
 */
uint8_t lis3dh_read_id(I2C_TypeDef *i2c_dev);

/**
 * \brief  Configure Sensor Interrupts (Threshold, Click, etc.).
 * \param  i2c_dev: I2C Peripheral instance.
 */
void lis3dh_config_interrupt(I2C_TypeDef *i2c_dev);

/**
 * \brief  Configure FIFO mode.
 */
void lis3dh_config_fifo(I2C_TypeDef *i2c_dev, bool enable, uint8_t threshold);

/**
 * \brief  Configure Burst Read.
 *         Configures I2C Wrapper for repeated reading.
 */
void lis3dh_config_burst_read(I2C_TypeDef *i2c_dev, uint8_t reg, uint32_t len);

#endif /* LIS3DH_H */
