/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: LicenseRef-Realtek-5-Clause
 */

/* Includes ------------------------------------------------------------------*/
#include "lis3dh_driver.h"

/* Defines -------------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/

/**
 * \brief  Internal helper: Write Register.
 */
static void lis3dh_reg_write(I2C_TypeDef *i2c_dev, uint8_t reg, uint8_t value)
{
    /* Disable DMA Mode during configuration */
    I2C_DMACmd(i2c_dev, I2C_DMA_REQ_RX, DISABLE);

    /* Use I2C Wrapper Mode */
    I2C_WrapperModeCmd(i2c_dev, ENABLE);
    I2C_WrapperSetTransMode(i2c_dev, I2C_WRAPPER_TRANS_MODE_WRITE);
    I2C_WrapperClearTxFIFO(i2c_dev);

    /*
     * SUB[7] is set as 0 in this API.
     * SUB[7] is used to enable address auto increment.
     * If the SUB[7] equals to 1, the SUB[0:6] (register address) is
     * automatically increased to allow multiple data read/writes.
     */

    I2C_WrapperSetWriteNum(i2c_dev, 2);
    I2C_WrapperSetWriteData(i2c_dev, &reg, 1);
    I2C_WrapperSetWriteData(i2c_dev, &value, 1);

    /* Start I2C transfer */
    I2C_ActionTrigger(i2c_dev, I2C_ACTION_START);

    /* Wait for I2C Transfer Done */
    while (I2C_GetFlagState(i2c_dev, I2C_FLAG_ACTIVITY)) {;}
}

/**
 * \brief  Internal helper: Read Register.
 */
static uint8_t lis3dh_reg_read(I2C_TypeDef *i2c_dev, uint8_t reg)
{
    /* Disable DMA Mode */
    I2C_DMACmd(i2c_dev, I2C_DMA_REQ_RX, DISABLE);

    /* Use I2C Wrapper Mode */
    I2C_WrapperModeCmd(i2c_dev, ENABLE);
    I2C_WrapperSetTransMode(i2c_dev, I2C_WRAPPER_TRANS_MODE_REPEAT_READ);
    I2C_WrapperClearTxFIFO(i2c_dev);

    /*
     * SUB[7] is set as 0 in this API.
     * SUB[7] is used to enable address auto increment.
     * If the SUB[7] equals to 1, the SUB[0:6] (register address) is
     * automatically increased to allow multiple data read/writes.
     */

    I2C_WrapperSetWriteNum(i2c_dev, 1);
    I2C_WrapperSetWriteData(i2c_dev, &reg, 1);
    I2C_WrapperSetReadNum(i2c_dev, 1);

    /* Start I2C transfer */
    I2C_ActionTrigger(i2c_dev, I2C_ACTION_START);

    /* Wait for I2C Transfer Done */
    while (I2C_GetFlagState(i2c_dev, I2C_FLAG_ACTIVITY)) {;}

    /* Receive data */
    return I2C_WrapperReceiveData(i2c_dev);
}

/**
 * \brief  Configure I2C for Burst Read.
 *         Note: Does not trigger START, waits for external signal (RAP/DMA).
 */
void lis3dh_config_burst_read(I2C_TypeDef *i2c_dev, uint8_t reg, uint32_t len)
{
    /* Enable RX DMA Request */
    I2C_DMACmd(i2c_dev, I2C_DMA_REQ_RX, ENABLE);

    /* Use I2C Wrapper Mode */
    I2C_WrapperModeCmd(i2c_dev, ENABLE);
    I2C_WrapperSetTransMode(i2c_dev, I2C_WRAPPER_TRANS_MODE_REPEAT_READ);
    I2C_WrapperClearTxFIFO(i2c_dev);

    /* Prepare Address (reg should include Auto-Increment Bit if needed) */
    I2C_WrapperSetWriteNum(i2c_dev, 1);
    I2C_WrapperSetWriteData(i2c_dev, &reg, 1);

    /* Set expected read length */
    I2C_WrapperSetReadNum(i2c_dev, len);
}

uint8_t lis3dh_read_id(I2C_TypeDef *i2c_dev)
{
    return lis3dh_reg_read(i2c_dev, LIS3DH_REG_WHO_AM_I);
}

void lis3dh_config_fifo(I2C_TypeDef *i2c_dev, bool enable, uint8_t threshold)
{
    if (enable)
    {
        /* Enable FIFO, Stream Mode, Set Threshold */
        uint8_t fifo_ctrl_reg = lis3dh_reg_read(i2c_dev, LIS3DH_REG_FIFO_CTRL_REG);
        fifo_ctrl_reg = (fifo_ctrl_reg & ~0xC0) | (LIS3DH_FIFO_MODE_STREAM_FIFO << 6);
        fifo_ctrl_reg &= ~BIT5; /* Clear TR bit */
        fifo_ctrl_reg |= (threshold & 0x1F);
        lis3dh_reg_write(i2c_dev, LIS3DH_REG_FIFO_CTRL_REG, fifo_ctrl_reg);

        /* Enable FIFO in Ctrl Reg 5 */
        uint8_t ctrl_reg5 = lis3dh_reg_read(i2c_dev, LIS3DH_REG_CTRL_REG5);
        ctrl_reg5 |= BIT6;
        lis3dh_reg_write(i2c_dev, LIS3DH_REG_CTRL_REG5, ctrl_reg5);
    }
    else
    {
        /* Bypass Mode */
        uint8_t fifo_ctrl_reg = lis3dh_reg_read(i2c_dev, LIS3DH_REG_FIFO_CTRL_REG);
        fifo_ctrl_reg = (fifo_ctrl_reg & ~0xC0) | (LIS3DH_FIFO_MODE_BYPASS << 6);
        lis3dh_reg_write(i2c_dev, LIS3DH_REG_FIFO_CTRL_REG, fifo_ctrl_reg);

        /* Disable FIFO in Ctrl Reg 5 */
        uint8_t ctrl_reg5 = lis3dh_reg_read(i2c_dev, LIS3DH_REG_CTRL_REG5);
        ctrl_reg5 &= ~BIT6;
        lis3dh_reg_write(i2c_dev, LIS3DH_REG_CTRL_REG5, ctrl_reg5);
    }
}

void lis3dh_config_interrupt(I2C_TypeDef *i2c_dev)
{
    /********************* INT1 CFG *********************/
    /*
     * INT1 CFG:
     * 0x30[6:7] AOI-6D is set as '10' means AND combination of interrupt events
     *  - AOI: 1 ~ AND, 0 ~ OR
     *  - 6D : 1  ~Direction Source, 0 ~ Interrupt Source
     * 0x30[0:5] Enable interrupt generation on X(H/L) Y(H/L) Z(H/L)
     */
    lis3dh_reg_write(i2c_dev, LIS3DH_REG_INT1_CFG, 0xBF);

    /*
     * INT1 THS:
     * 0x32[0:6] Interrupt threshold support 2^7=128 prescaler of max range.
     */
    lis3dh_reg_write(i2c_dev, LIS3DH_REG_INT1_THS, 0x10); /* 1/8 range */

    /*
     * INT1 DURAION:
     * 0x33[0:6] Interrupt Duration.
     */
    lis3dh_reg_write(i2c_dev, LIS3DH_REG_INT1_DURATION, 0x01); /* 1 * 1/50 s = 20ms */

    /********************* Click CFG *********************/
    /*
     * CLICK CFG:
     * 0x38[0:5] Enable interrupt single/double click on X(H/L) Y(H/L) Z(H/L)
     */
    lis3dh_reg_write(i2c_dev, LIS3DH_REG_CLICK_CFG, 0x15);/* 0x2A is for Double Click */

    /*
     * CLICK SRC:
     * 0x39[0:6] Enable interrupt source
     * - 0x39[0]: X click detection
     * - 0x39[1]: Y click detection
     * - 0x39[2]: Z click detection
     * - 0x39[3]: Click Sign
     * - 0x39[4]: Single Click
     * - 0x39[5]: Double Click
     * - 0x39[6]: Interrupt Active
     */
    //lis3dh_reg_write(i2c_dev, LIS3DH_REG_CLICK_SRC, 0x17);

    /*
     * CLICK THS:
     * 0x3A[0:6] Click thresold.
     * This sets the threshold where the click detection process is activated.
     */
    lis3dh_reg_write(i2c_dev, LIS3DH_REG_CLICK_THS, 0x0A); /* 1/16 range */

    /*
     * TIME LIMIT:
     * 0x3B[0:6] Click time limit
     * Time acceleration has to fall below threshold for a valid click.
     */
    lis3dh_reg_write(i2c_dev, LIS3DH_REG_TIME_LIMIT, 0x08); /* 8 * 1/50 s = 160ms */

    /*
     * TIME LATENCY:
     * 0x3C[0:7] Click time latency
     * hold-off time before allowing detection after click event
     */
    lis3dh_reg_write(i2c_dev, LIS3DH_REG_TIME_LATENCY, 0x08); /* 8 * 1/50 s = 160ms */

    /*
     * TIME WINDOWN:
     * 0x3D[0:7] Click time window
     * hold-off time before allowing detection after click event
     */
    lis3dh_reg_write(i2c_dev, LIS3DH_REG_TIME_WINDOW, 0x10); /* 16 * 1/50 s = 320ms */

    /********************* Basic CFG *********************/
    /* Clear Latch Interrupt and 4 Direction Detection */
    uint8_t ctrl_reg5 = lis3dh_reg_read(i2c_dev, LIS3DH_REG_CTRL_REG5);
    ctrl_reg5 &= 0xF3;
    ctrl_reg5 &= ~0x08;
    lis3dh_reg_write(i2c_dev, LIS3DH_REG_CTRL_REG5, ctrl_reg5);

    /*
     * CTRL REG3:
     * 0x22[0:7] Interrupt Configure
     * - 0x22[0]: -
     * - 0x22[1]: FIFO Overrun on INT1
     * - 0x22[2]: FIFO Watermack on INT1
     * - 0x22[3]: 3 2 1 DATA ??
     * - 0x22[4]: X Y Z DATA (Data Ready) on INT1
     * - 0x22[5]: IA2 Interrupt on INT1
     * - 0x22[6]: IA1 Interrupt on INT1
     * - 0x22[7]: Click Interrupt on INT1
     */
    uint8_t ctrl_reg3 = 0x60; /* AOI1 | AOI2 */
    lis3dh_reg_write(i2c_dev, LIS3DH_REG_CTRL_REG3, ctrl_reg3);

    /*
     * CTRL REG6:
     * 0x25[0:7] Interrupt Configure
     * - 0x25[0]: -
     * - 0x25[1]: INT1 and INT2 pin polarity
     * - 0x25[2]: -
     * - 0x25[3]: Activity Interrupt on INT2
     * - 0x25[4]: Boot on INT2
     * - 0x25[5]: IA2 Interrupt on INT2
     * - 0x25[6]: IA1 Interrupt on INT2
     * - 0x25[7]: Click Interrupt on INT2
     */
    uint8_t ctrl_reg6 = 0x80; /* Click */
    lis3dh_reg_write(i2c_dev, LIS3DH_REG_CTRL_REG6, ctrl_reg6);

    /* Overwrite CTRL REG3 */
    ctrl_reg3 = LIS3DH_I1_WTM;
    lis3dh_reg_write(i2c_dev, LIS3DH_REG_CTRL_REG3, ctrl_reg3);
}

void lis3dh_init(I2C_TypeDef *i2c_dev)
{
    /*
     * CTRL REG1:
     * 0x20[0:7] Basic Configure
     * - 0x20[0]: X-axis enable
     * - 0x20[1]: Y-axis enable
     * - 0x20[2]: X-axis enable
     * - 0x20[3]: Low Power Mode enable
     * - 0x20[4~7]: Data Rate Selection
     *
     * Enable X Y Z Axis and config Data Rate as 50Hz
     */
    uint8_t ctrl_reg1 = (LIS3DH_DATA_RATE_50_HZ << 4) | 0x07;
    lis3dh_reg_write(i2c_dev, LIS3DH_REG_CTRL_REG1, ctrl_reg1);

    /*
     * CTRL REG4:
     * 0x23[0:7] Basic Configure
     * - 0x23[0]: SPI serial interface mode selection
     * - 0x23[1:2]: Self-test enable.
     * - 0x23[3]: High-resolution output mode
     * - 0x23[4:5]: Full-scale selection(00: 2g; 01: 4g; 10: 8g; 11: 16g)
     * - 0x23[6]: Big/little endian data selection.
     * - 0x23[7]: Block data update
     *
     * Use +/- 2g scale, High-resolution, Block data update
     */
    uint8_t ctrl_reg4 = (0x00 << 4) | 0x80 | 0x08;
    lis3dh_reg_write(i2c_dev, LIS3DH_REG_CTRL_REG4, ctrl_reg4);

    /* Disable FIFO Mode Firstly */
    lis3dh_config_fifo(i2c_dev, DISABLE, 0);
}
