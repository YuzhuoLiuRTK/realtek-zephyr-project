/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: LicenseRef-Realtek-5-Clause
 */

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <string.h>
#include "rtl_rcc.h"
#include "rtl_pinmux.h"
#include "rtl_nvic.h"
#include "rtl_i2c.h"
#include "rtl_timer.h"
#include "rtl_rap.h"
#include "log_core.h"

/* Defines -------------------------------------------------------------------*/
/* I2C configuration which can be modified based on requirements */
/* Configure I2C Master parameters */
#define I2C_MASTER                  I2C0
#define I2C_MASTER_CLOCK            I2C0_CLOCK
#define I2C_MASTER_SCL_PIN          P4_0
#define I2C_MASTER_SDA_PIN          P4_1
#define I2C_MASTER_SCL_PINMUX       I2C0_CLK
#define I2C_MASTER_SDA_PINMUX       I2C0_DAT

/* Configure I2C Slave parameters */
#define I2C_SLAVE                   I2C1
#define I2C_SLAVE_IRQN              I2C_1_IRQn
#define I2C_SLAVE_CLOCK             I2C1_CLOCK
#define I2C_SLAVE_SCL_PIN           P4_2
#define I2C_SLAVE_SDA_PIN           P4_3
#define I2C_SLAVE_SCL_PINMUX        I2C1_CLK
#define I2C_SLAVE_SDA_PINMUX        I2C1_DAT

/* Configure I2C Parameters */
#define I2C_SPEED                   100000
#define I2C_SLAVE_ADDR              0x50
#define I2C_MASTER_WRITE_LEN        10

/* RAP Configuration */
#define I2C_ACTION_START            RAP_ACTION_I2C_START(0)

/* TIMER configuration which can be modified based on requirements */
#define TIMER_NUM                   TIMER1_CH0
#define TIMER_IRQN                  TIMER1_CH0_IRQn

/*
 * Configure TIMER Period parameters.
 *
 * Calculation formula:
 *  - Timeout = Period * Tick(Clock_Div/Clock_Src)
 * The formula is equivalent to:
 *  - Period = Timeout_ms * (Clock_Src / Clock_Div / 1000).
 *
 * Based on the following settings:
 *  - TIMER Clock Source: 40MHz
 *  - TIMER Clock Divide: DIV_1.
 *
 * Define the TIMER_PERIOD as 40000000 which timeout is 1 second.
 */
#define TIMER_PERIOD                40000000

/* RAP Configuration */
#define TIMER_EVENT_TIMEOUT         RAP_EVENT_TIMER_TIMEOUT(1, 0)

/* Globals -------------------------------------------------------------------*/
static uint8_t i2c_master_tx_buffer[I2C_MASTER_WRITE_LEN] = {0};
static uint8_t i2c_slave_rx_buffer[256];
static volatile uint16_t i2c_slave_rx_length = 0;
volatile bool is_receive_done = false;

/* Functions -----------------------------------------------------------------*/

/**
 * \brief  TIMER Interrupt Service Routine (ISR) prototype.
 */
static void TIMER_Handler(void);

/**
 * \brief  I2C Slave Interrupt Service Routine (ISR) prototype.
 */
static void I2C_SLAVE_Handler(void);

/**
 * \brief  Initialize I2C Master pad and pinmux settings.
 */
static void board_i2c_master_init(void)
{
    Pad_Config(I2C_MASTER_SCL_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE,
               PAD_OUT_HIGH);
    Pad_Config(I2C_MASTER_SDA_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE,
               PAD_OUT_HIGH);

    Pinmux_Config(I2C_MASTER_SCL_PIN, I2C_MASTER_SCL_PINMUX);
    Pinmux_Config(I2C_MASTER_SDA_PIN, I2C_MASTER_SDA_PINMUX);
}

/**
 * \brief  Initialize I2C Slave pad and pinmux settings.
 */
static void board_i2c_slave_init(void)
{
    Pad_Config(I2C_SLAVE_SCL_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE,
               PAD_OUT_HIGH);
    Pad_Config(I2C_SLAVE_SDA_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE,
               PAD_OUT_HIGH);

    Pinmux_Config(I2C_SLAVE_SCL_PIN, I2C_SLAVE_SCL_PINMUX);
    Pinmux_Config(I2C_SLAVE_SDA_PIN, I2C_SLAVE_SDA_PINMUX);
}

/**
 * \brief  Initialize I2C Master peripheral.
 */
static void driver_i2c_master_init(void)
{
    /* Enable I2C Master clock */
    RCC_ClockCmd(I2C_MASTER_CLOCK, ENABLE);

    /* Configure I2C Master parameters */
    I2C_InitTypeDef I2C_InitStruct;
    I2C_StructInit(&I2C_InitStruct);
    I2C_InitStruct.I2C_ClockSpeed    = I2C_SPEED;
    I2C_InitStruct.I2C_DeviceMode    = I2C_DEVICE_MODE_MASTER;
    I2C_InitStruct.I2C_AddressMode   = I2C_ADDRESS_MODE_7BIT;
    I2C_InitStruct.I2C_SlaveAddress  = I2C_SLAVE_ADDR;
    I2C_InitStruct.I2C_Ack           = ENABLE;
    I2C_Init(I2C_MASTER, &I2C_InitStruct);

    /* Enable I2C */
    I2C_Cmd(I2C_MASTER, ENABLE);
}

/**
 * \brief  Initialize I2C Slave peripheral.
 */
static void driver_i2c_slave_init(void)
{
    /* Enable I2C Slave clock */
    RCC_ClockCmd(I2C_SLAVE_CLOCK, ENABLE);

    /* Configure I2C Slave parameters */
    I2C_InitTypeDef I2C_InitStruct;
    I2C_StructInit(&I2C_InitStruct);
    I2C_InitStruct.I2C_ClockSpeed        = I2C_SPEED;
    I2C_InitStruct.I2C_DeviceMode        = I2C_DEVICE_MODE_SLAVE;
    I2C_InitStruct.I2C_AddressMode       = I2C_ADDRESS_MODE_7BIT;
    I2C_InitStruct.I2C_SlaveAddress      = I2C_SLAVE_ADDR;
    I2C_InitStruct.I2C_Ack               = ENABLE;
    I2C_InitStruct.I2C_RxThresholdLevel  = 4;
    I2C_Init(I2C_SLAVE, &I2C_InitStruct);

    /* Update vector table with ISR */
    ram_vector_table_update(I2C_SLAVE_IRQN, I2C_SLAVE_Handler);

    /* Configure NVIC */
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel         = I2C_SLAVE_IRQN;
    NVIC_InitStruct.NVIC_IRQChannelCmd      = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_Init(&NVIC_InitStruct);

    /* Enable RX FIFO threshold and Stop Detect interrupts */
    I2C_ClearINTPendingBit(I2C_SLAVE, I2C_INT_RX_FULL);
    I2C_ClearINTPendingBit(I2C_SLAVE, I2C_INT_STOP_DET);
    I2C_INTConfig(I2C_SLAVE, I2C_INT_RX_FULL, ENABLE);
    I2C_INTConfig(I2C_SLAVE, I2C_INT_STOP_DET, ENABLE);

    /* Enable I2C */
    I2C_Cmd(I2C_SLAVE, ENABLE);
}

/**
 * \brief  Initialize TIMER peripheral.
 */
static void driver_timer_init(void)
{
    /* Enable TIMER clock */
    RCC_ClockCmd(TIMER1_CLOCK, ENABLE);

    /* Configure TIMER parameters */
    TIMER_TimeBaseInitTypeDef TIMER_InitStruct;
    TIMER_StructInit(&TIMER_InitStruct);
    TIMER_InitStruct.TIMER_ClockSrc = TIMER_CLOCK_SRC_40M;
    TIMER_InitStruct.TIMER_ClockDiv = TIMER_CLOCK_DIV_1;
    TIMER_InitStruct.PWM_En         = DISABLE;
    TIMER_InitStruct.TIMER_Mode     = TIMER_MODE_USERDEFINE_AUTO;
    TIMER_InitStruct.TIMER_Period   = TIMER_PERIOD;
    TIMER_TimeBaseInit(TIMER_NUM, &TIMER_InitStruct);

    /*
     * When the trigger conditions are met, the system supports the simultaneous
     * triggering of RAP events and interrupts. Users can enable the interrupt
     * function according to their actual usage needs.
     * Set to 1 to activate the interrupt function.
     */
#if 0
    /* Update vector table with ISR */
    ram_vector_table_update(TIMER_IRQN, TIMER_Handler);

    /* Configure NVIC */
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel         = TIMER_IRQN;
    NVIC_InitStruct.NVIC_IRQChannelCmd      = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_Init(&NVIC_InitStruct);

    /* Enable TIMER Interrupt */
    TIMER_ClearINT(TIMER_NUM, TIMER_INT_TIMEOUT);
    TIMER_INTConfig(TIMER_NUM, TIMER_INT_TIMEOUT, ENABLE);
#endif
}

/**
 * \brief  Configure I2C Master Wrapper mode for RAP Trigger.
 */
static void driver_i2c_master_wrapper_config(void)
{
    /* Initialize I2C Master TX buffer */
    for (uint8_t i = 0; i < I2C_MASTER_WRITE_LEN; i++)
    {
        i2c_master_tx_buffer[i] = i;
    }

    /* Configure I2C Wrapper Mode */
    I2C_WrapperModeCmd(I2C_MASTER, ENABLE);
    I2C_WrapperSetTransMode(I2C_MASTER, I2C_WRAPPER_TRANS_MODE_WRITE);
    I2C_WrapperClearTxFIFO(I2C_MASTER);
    I2C_WrapperSetWriteNum(I2C_MASTER, I2C_MASTER_WRITE_LEN);
    I2C_WrapperSetWriteData(I2C_MASTER, i2c_master_tx_buffer, I2C_MASTER_WRITE_LEN);
}

/**
 * \brief  Main entry.
 */
int main(void)
{
    /* Enable Global Interrupts */
    __enable_irq();

    DBG_DIRECT("Start RAP timer trigger I2C master write sample");

    /* Peripheral board initialization */
    board_i2c_master_init();
    board_i2c_slave_init();

    /* Peripheral driver initialization */
    driver_timer_init();
    driver_i2c_master_init();
    driver_i2c_slave_init();

    /* Configure I2C Master Wrapper for RAP */
    driver_i2c_master_wrapper_config();

    /* Configure RAP channel */
    uint8_t channel;
    RAP_ChannelAllocate(&channel);

    /* Route TIMER Timeout Event to RAP channel */
    RAP_EventRouteSet(TIMER_EVENT_TIMEOUT, channel);
    /* Bind I2C Start Action to RAP channel */
    RAP_ActionBindSet(I2C_ACTION_START, channel);

    /* Enable RAP Mode for TIMER and I2C */
    TIMER_RAPModeCmd(TIMER_NUM, ENABLE);
    I2C_RAPModeCmd(I2C_MASTER, ENABLE);

    /* Start TIMER to trigger I2C Write via RAP */
    TIMER_ActionTrigger(TIMER_NUM, TIMER_ACTION_START);

    while (1)
    {
        /*
         * Check I2C Slave reception status.
         * Process data in main loop instead of ISR for better performance.
         */
        if (is_receive_done)
        {
            /* Clear flag immediately */
            is_receive_done = false;

            /* Print received data information */
            DBG_DIRECT("I2C Slave RX Length = %d", i2c_slave_rx_length);

            /* Verify and print data */
            for (uint32_t i = 0; i < i2c_slave_rx_length; i++)
            {
                if (i2c_master_tx_buffer[i] != i2c_slave_rx_buffer[i])
                {
                    DBG_DIRECT(" Data transmission error! Master TX Data[%d]: %d, Slave RX Data[%d]: %d",
                               i, (uint8_t)i2c_master_tx_buffer[i], i, i2c_slave_rx_buffer[i]);
                }
                DBG_DIRECT("Master TX Data[%d]: %d, Slave RX Data[%d]: %d",
                           i, (uint8_t)i2c_master_tx_buffer[i], i, i2c_slave_rx_buffer[i]);
            }

            /* Reset buffer and length for next transaction */
            i2c_slave_rx_length = 0;
            memset(i2c_slave_rx_buffer, 0, sizeof(i2c_slave_rx_buffer));
        }
    }
}

/**
 * \brief  TIMER Interrupt Service Routine (ISR).
 */
static void TIMER_Handler(void)
{
    DBG_DIRECT("TIMER_Handler");

    if (TIMER_GetINTStatus(TIMER_NUM, TIMER_INT_TIMEOUT) == SET)
    {
        TIMER_Cmd(TIMER_NUM, DISABLE);
        TIMER_ClearINT(TIMER_NUM, TIMER_INT_TIMEOUT);

        /* Add Test Code here */
        TIMER_Cmd(TIMER_NUM, ENABLE);
    }
}

/**
 * \brief  I2C Slave Interrupt Service Routine (ISR).
 */
static void I2C_SLAVE_Handler(void)
{
    /* Check Stop Signal Detect Interrupt */
    if (I2C_GetINTStatus(I2C_SLAVE, I2C_INT_STOP_DET) == SET)
    {
        DBG_DIRECT("I2C_SLAVE: I2C_INT_STOP_DET");

        /* Read remaining data in RX FIFO */
        uint16_t fifo_len = I2C_GetRxFIFOLen(I2C_SLAVE);
        for (uint32_t i = 0; i < fifo_len; i++)
        {
            i2c_slave_rx_buffer[i2c_slave_rx_length++] = I2C_ReceiveData(I2C_SLAVE);
        }

        /* Clear interrupt pending bit */
        I2C_ClearINTPendingBit(I2C_SLAVE, I2C_INT_STOP_DET);

        /* Set flag to notify main loop */
        is_receive_done = true;
    }

    /* Check RX FIFO Full Interrupt */
    if (I2C_GetINTStatus(I2C_SLAVE, I2C_INT_RX_FULL) == SET)
    {
        DBG_DIRECT("I2C_SLAVE: I2C_INT_RX_FULL");

        /* Read data from RX FIFO */
        uint16_t fifo_len = I2C_GetRxFIFOLen(I2C_SLAVE);
        for (uint32_t i = 0; i < fifo_len; i++)
        {
            i2c_slave_rx_buffer[i2c_slave_rx_length++] = I2C_ReceiveData(I2C_SLAVE);
        }

        /* Clear interrupt pending bit */
        I2C_ClearINTPendingBit(I2C_SLAVE, I2C_INT_RX_FULL);
    }
}
