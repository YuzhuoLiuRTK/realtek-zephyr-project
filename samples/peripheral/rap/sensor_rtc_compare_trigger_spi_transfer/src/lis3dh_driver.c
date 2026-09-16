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
static void lis3dh_reg_write(SPI_TypeDef *spi_dev, uint8_t reg, uint8_t value)
{
    /* Prepare Command: Write (RW=0) */
    uint8_t cmd = reg & (~LIS3DH_SPI_RW_BIT);

    /* Use CPU so disable RAP and RX DMA */
    SPI_RAPModeCmd(spi_dev, DISABLE);
    SPI_DMACmd(spi_dev, SPI_DMA_REQ_RX, DISABLE);

    /*
     * SUB[6] is set as 0 in this API.
     * SUB[6] is used to enable address auto increment.
     * If the SUB[6] equals to 1, the SUB[0:6] (register address) is
     * automatically increased to allow multiple data read/writes.
     */

    SPI_SetDirection(spi_dev, SPI_DIRECTION_TXONLY);

    /* TX NDF = 1 (Cmd) + 1 (Data) */
    SPI_WrapModeSetTxNdf(spi_dev, 2);
    SPI_WrapModeResetTxFIFO(spi_dev);
    SPI_SendBuffer(spi_dev, &cmd, 1);
    SPI_SendBuffer(spi_dev, &value, 1);
    SPI_WrapModeStartTx(spi_dev, ENABLE);

    while (SPI_GetFlagState(spi_dev, SPI_FLAG_TFE) == 0) {;}
    while (SPI_GetFlagState(spi_dev, SPI_FLAG_WRAP_CS_EN) == 1) {;}
}

/**
 * \brief  Internal helper: Read Register.
 */
static uint8_t lis3dh_reg_read(SPI_TypeDef *spi_dev, uint8_t reg)
{
    /* Internal buffer for single register reads */
    uint8_t internal_read_buf[16] = {0};

    /* Prepare Command: Read (RW=1) */
    uint8_t cmd = reg | LIS3DH_SPI_RW_BIT;

    /* Use CPU so disable RAP and RX DMA */
    SPI_RAPModeCmd(spi_dev, DISABLE);
    SPI_DMACmd(spi_dev, SPI_DMA_REQ_RX, DISABLE);

    /*
     * SUB[6] is set as 0 in this API.
     * SUB[6] is used to enable address auto increment.
     * If the SUB[6] equals to 1, the SUB[0:6] (register address) is
     * automatically increased to allow multiple data read/writes.
     */

    SPI_SetDirection(spi_dev, SPI_DIRECTION_FULLDUPLEX);

    /* TX NDF = 1 (Cmd) + 1 (Data) */
    SPI_WrapModeSetTxNdf(spi_dev, 2);
    SPI_WrapModeResetTxFIFO(spi_dev);
    SPI_SendBuffer(spi_dev, &cmd, 1);
    SPI_SendBuffer(spi_dev, internal_read_buf, 1); /* Buffer used as dummy source here */
    SPI_WrapModeStartTx(spi_dev, ENABLE);

    while (SPI_GetFlagState(spi_dev, SPI_FLAG_RFNE) == 0) {;}
    while (SPI_GetFlagState(spi_dev, SPI_FLAG_WRAP_CS_EN) == 1) {;}

    /* Read RX FIFO */
    SPI_ReceiveData(spi_dev); /* Discard dummy byte corresponding to command phase */
    return SPI_ReceiveData(spi_dev); /* Return actual data */
}

/**
 * \brief  Configure SPI for Burst Read (RAP Triggered).
 *         Configures the SPI Protocol (NDF, TransferNum) and Enables RAP.
 */
void lis3dh_config_burst_read(SPI_TypeDef *spi_dev, uint8_t reg, uint32_t len)
{
    /* Enable RX DMA Request on SPI side */
    SPI_DMACmd(spi_dev, SPI_DMA_REQ_RX, ENABLE);

    SPI_SetDirection(spi_dev, SPI_DIRECTION_FULLDUPLEX);

    /*
     * TX NDF = len + 1.
     * 1 byte for Command Phase, 'len' bytes for Data Phase.
     */
    SPI_WrapModeSetTxNdf(spi_dev, len + 1);
    SPI_WrapModeResetTxFIFO(spi_dev);
    /* Set Action Parameters */
    SPI_SetActionCmdNum(spi_dev, 1);
    SPI_SetActionTransferNum(spi_dev, len);

    /* Manual Mode: Pre-fill FIFO manually with the Command Byte */
    SPI_SendBuffer(spi_dev, &reg, 1);
}


uint8_t lis3dh_read_id(SPI_TypeDef *spi_dev)
{
    return lis3dh_reg_read(spi_dev, LIS3DH_REG_WHO_AM_I);
}

void lis3dh_config_fifo(SPI_TypeDef *spi_dev, bool enable, uint8_t threshold)
{
    if (enable)
    {
        /* Enable FIFO, Stream Mode, Set Threshold */
        uint8_t fifo_ctrl_reg = lis3dh_reg_read(spi_dev, LIS3DH_REG_FIFO_CTRL_REG);
        fifo_ctrl_reg = (fifo_ctrl_reg & ~0xC0) | (LIS3DH_FIFO_MODE_STREAM_FIFO << 6);
        fifo_ctrl_reg &= ~BIT5;
        fifo_ctrl_reg |= (threshold & 0x1F);
        lis3dh_reg_write(spi_dev, LIS3DH_REG_FIFO_CTRL_REG, fifo_ctrl_reg);

        /* Enable FIFO Mode */
        uint8_t ctrl_reg5 = lis3dh_reg_read(spi_dev, LIS3DH_REG_CTRL_REG5);
        ctrl_reg5 |= BIT6;
        lis3dh_reg_write(spi_dev, LIS3DH_REG_CTRL_REG5, ctrl_reg5);
    }
    else
    {
        /* Configure FIFO mode as Bypass Mode */
        uint8_t fifo_ctrl_reg = lis3dh_reg_read(spi_dev, LIS3DH_REG_FIFO_CTRL_REG);
        fifo_ctrl_reg = (fifo_ctrl_reg & ~0xC0) | (LIS3DH_FIFO_MODE_BYPASS << 6);
        lis3dh_reg_write(spi_dev, LIS3DH_REG_FIFO_CTRL_REG, fifo_ctrl_reg);

        /* Disable FIFO Mode */
        uint8_t ctrl_reg5 = lis3dh_reg_read(spi_dev, LIS3DH_REG_CTRL_REG5);
        ctrl_reg5 &= ~BIT6;
        lis3dh_reg_write(spi_dev, LIS3DH_REG_CTRL_REG5, ctrl_reg5);
    }
}

void lis3dh_config_interrupt(SPI_TypeDef *spi_dev)
{
    /********************* INT1 CFG *********************/
    /*
     * INT1 CFG:
     * 0x30[6:7] AOI-6D is set as '10' means AND combination of interrupt events
     *  - AOI: 1 ~ AND, 0 ~ OR
     *  - 6D : 1  ~Direction Source, 0 ~ Interrupt Source
     * 0x30[0:5] Enable interrupt generation on X(H/L) Y(H/L) Z(H/L)
     */
    lis3dh_reg_write(spi_dev, LIS3DH_REG_INT1_CFG, 0xBF);

    /*
     * INT1 THS:
     * 0x32[0:6] Interrupt threshold support 2^7=128 prescaler of max range.
     */
    lis3dh_reg_write(spi_dev, LIS3DH_REG_INT1_THS, 0x10); /* 1/8 range */

    /*
     * INT1 DURAION:
     * 0x33[0:6] Interrupt Duration.
     */
    lis3dh_reg_write(spi_dev, LIS3DH_REG_INT1_DURATION, 0x01); /* 1 * 1/50 s = 20ms */

    /********************* Click CFG *********************/
    /*
     * CLICK CFG:
     * 0x38[0:5] Enable interrupt single/double click on X(H/L) Y(H/L) Z(H/L)
     */
    lis3dh_reg_write(spi_dev, LIS3DH_REG_CLICK_CFG, 0x15);/* 0x2A is for Double Click */

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
    //lis3dh_reg_write(spi_dev, LIS3DH_REG_CLICK_SRC, 0x17);

    /*
     * CLICK THS:
     * 0x3A[0:6] Click thresold.
     * This sets the threshold where the click detection process is activated.
     */
    lis3dh_reg_write(spi_dev, LIS3DH_REG_CLICK_THS, 0x0A); /* 1/16 range */

    /*
     * TIME LIMIT:
     * 0x3B[0:6] Click time limit
     * Time acceleration has to fall below threshold for a valid click.
     */
    lis3dh_reg_write(spi_dev, LIS3DH_REG_TIME_LIMIT, 0x08); /* 8 * 1/50 s = 160ms */

    /*
     * TIME LATENCY:
     * 0x3C[0:7] Click time latency
     * hold-off time before allowing detection after click event
     */
    lis3dh_reg_write(spi_dev, LIS3DH_REG_TIME_LATENCY, 0x08); /* 8 * 1/50 s = 160ms */

    /*
     * TIME WINDOWN:
     * 0x3D[0:7] Click time window
     * hold-off time before allowing detection after click event
     */
    lis3dh_reg_write(spi_dev, LIS3DH_REG_TIME_WINDOW, 0x10); /* 16 * 1/50 s = 320ms */

    /********************* Basic CFG *********************/
    /* Clear Latch Interrupt and 4 Direction Detection */
    uint8_t ctrl_reg5 = lis3dh_reg_read(spi_dev, LIS3DH_REG_CTRL_REG5);
    ctrl_reg5 &= ~0x08; /* Do not latch interrupt */
    lis3dh_reg_write(spi_dev, LIS3DH_REG_CTRL_REG5, ctrl_reg5);

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
    lis3dh_reg_write(spi_dev, LIS3DH_REG_CTRL_REG3, ctrl_reg3);

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
    lis3dh_reg_write(spi_dev, LIS3DH_REG_CTRL_REG6, ctrl_reg6);

    /*
     * Overwrite CTRL REG3
     * Matches original code behavior exactly (enabling FIFO Watermark INT on PIN1
     */
    ctrl_reg3 = LIS3DH_I1_WTM;
    lis3dh_reg_write(spi_dev, LIS3DH_REG_CTRL_REG3, ctrl_reg3);
}

void lis3dh_init(SPI_TypeDef *spi_dev)
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
    lis3dh_reg_write(spi_dev, LIS3DH_REG_CTRL_REG1, ctrl_reg1);

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
    lis3dh_reg_write(spi_dev, LIS3DH_REG_CTRL_REG4, ctrl_reg4);

    /* Disable FIFO Mode Firstly */
    lis3dh_config_fifo(spi_dev, DISABLE, 0);
}
