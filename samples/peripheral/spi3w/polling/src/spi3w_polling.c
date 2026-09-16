/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: LicenseRef-Realtek-5-Clause
 */

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <stdbool.h>
#include "rtl_rcc.h"
#include "rtl_pinmux.h"
#include "rtl_spi3w.h"
#include "log_core.h"

/* Defines -------------------------------------------------------------------*/
/* SPI3W Pin Configuration */
#define SPI3W_CLK_PIN                   P4_0
#define SPI3W_DATA_PIN                  P4_1
#define SPI3W_CS_PIN                    P4_2

/*
 * Configure SPI3W Clock and Timing parameters.
 *
 * Read_Delay_Time = (Read_Delay + 1) / (2 * SPI3W_Speed).
 * Result: (3 + 1) / (2 * 800kHz) = 2.5us (Address phase to Data phase delay).
 */
#define SPI3W_SPEED                     800000
#define SPI3W_READ_DELAY                3

/*
 * Macro to handle busy wait loops with timeout logging.
 * If timeout occurs, it logs the line number and condition, then breaks the loop.
 * Execution continues to the next line (soft failure).
 */
#define SPI3W_WAIT_WHILE(cond) do {                          \
        uint32_t timeout = 0;                                \
        while (cond)                                         \
        {                                                    \
            if (++timeout > 0x1FFFF)                         \
            {                                                \
                DBG_DIRECT("Timeout Line is: %d", __LINE__); \
                break;                                       \
            }                                                \
        }                                                    \
    } while (0)

/* Globals -------------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/

/**
 * \brief  Initialize SPI3W pad and pinmux settings.
 */
void board_spi3w_init(void)
{
    Pad_Config(SPI3W_CLK_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE,
               PAD_OUT_HIGH);
    Pad_Config(SPI3W_DATA_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE,
               PAD_OUT_HIGH);
    Pad_Config(SPI3W_CS_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE,
               PAD_OUT_HIGH);

    Pinmux_Config(SPI3W_CLK_PIN, SPI3W_CLK_MASTER);
    Pinmux_Config(SPI3W_DATA_PIN, SPI3W_DATA_MASTER);
    Pinmux_Config(SPI3W_CS_PIN, SPI3W_CS_MASTER);
}

/**
 * \brief  Initialize SPI3W peripheral.
 */
void driver_spi3w_init(void)
{
    /* Enable SPI3W clock */
    RCC_ClockCmd(SPI3W_CLOCK, ENABLE);

    /* Configure SPI3W parameters */
    SPI3W_InitTypeDef SPI3W_InitStruct;
    SPI3W_StructInit(&SPI3W_InitStruct);
    SPI3W_InitStruct.SPI3W_SysClock     = 20000000;
    SPI3W_InitStruct.SPI3W_Speed        = SPI3W_SPEED;
    SPI3W_InitStruct.SPI3W_Mode         = SPI3W_3WIRE_MODE;
    SPI3W_InitStruct.SPI3W_ReadDelay    = SPI3W_READ_DELAY;
    SPI3W_InitStruct.SPI3W_OutputDelay  = SPI3W_OE_DELAY_NONE;
    SPI3W_InitStruct.SPI3W_ExtMode      = SPI3W_NORMAL_MODE;
    SPI3W_Init(&SPI3W_InitStruct);
}

/**
 * \brief  Read one byte through SPI3W peripheral.
 * \param  address: Address of register to read.
 * \return Value of register. Returns 0xFF if timeout occurs/read fails.
 */
uint8_t spi3w_read_byte(uint8_t address)
{
    uint8_t reg_value = 0xFF;

    /* Check busy before reading */
    SPI3W_WAIT_WHILE(SPI3W_GetFlagStatus(SPI3W_FLAG_BUSY) == SET);

    /* Clear RX data length before reading */
    SPI3W_ClearRxDataLen();

    /* Start Read */
    SPI3W_StartRead(address, 1);

    /* Check read command is write succesfully */
    SPI3W_WAIT_WHILE(SPI3W_GetFlagStatus(SPI3W_FLAG_BUSY) == SET);

    /* Check RX FIFO has received data */
    SPI3W_WAIT_WHILE(SPI3W_GetRxDataLen() == 0);

    /* Read data from RX FIFO */
    SPI3W_ReadBuf(&reg_value, 1);

    return reg_value;
}

/**
 * \brief  Write one byte through SPI3W peripheral.
 * \param  address: Address of register to write data to.
 * \param  data: Data to write.
 * \return true: write success, false: write failure.
 */
bool spi3w_write_byte(uint8_t address, uint8_t data)
{
    /* Check busy before writing */
    SPI3W_WAIT_WHILE(SPI3W_GetFlagStatus(SPI3W_FLAG_BUSY) == SET);

    /* Start Write */
    SPI3W_StartWrite(address, data);

    /* Check write data is completed */
    SPI3W_WAIT_WHILE(SPI3W_GetFlagStatus(SPI3W_FLAG_BUSY) == SET);

    return true;
}

/**
 * \brief  Read mouse product ID.
 * \param  p_id: Pointer to production id buffer.
 */
void mouse_get_product_id(uint8_t *p_id)
{
    *p_id++ = spi3w_read_byte(0x00);
    *p_id   = spi3w_read_byte(0x01);
}

/**
 * \brief  Main entry.
 */
int main(void)
{
    /* Enable Global Interrupts */
    __enable_irq();

    DBG_DIRECT("Start SPI3W polling sample");

    /* Peripheral initialization */
    board_spi3w_init();
    driver_spi3w_init();

    /* Enable SPI3W */
    SPI3W_Cmd(ENABLE);

    uint8_t id[2] = {0, 0};
    mouse_get_product_id(id);

    DBG_DIRECT("SPI3W Read ID: id[0] = 0x%x, id[1] = 0x%x", id[0], id[1]);

    if ((0x3E == id[0]) && (0x01 == (id[1] & 0xFF)))
    {
        DBG_DIRECT("SPI3W Read ID Pass");
    }
    else
    {
        DBG_DIRECT("SPI3W Read ID Fail");
    }

    while (1)
    {
    }

    return 0;
}
