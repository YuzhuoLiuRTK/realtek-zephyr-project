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
#include "rtl_gpio.h"
#include "rtl_timer.h"
#include "rtl_rap.h"
#include "rtl_spi3w.h"
#include "log_core.h"

/* Defines -------------------------------------------------------------------*/
/* SPI3W configuration which can be modified based on requirements */
/* Configure SPI3W Master parameters */
#define SPI3W_CLK_PIN               P4_0
#define SPI3W_DATA_PIN              P4_1
#define SPI3W_QB_PIN                P4_2
#define SPI3W_CLK_PINMUX            SPI3W_CLK_MASTER
#define SPI3W_DATA_PINMUX           SPI3W_DATA_MASTER
#define SPI3W_QB_PINMUX             SPI3W_QB_MASTER

/*
 * Configure SPI3W Clock and Timing parameters.
 *
 * Read_Delay_Time = (Read_Delay + 1) / (2 * SPI3W_Speed).
 * Result: (3 + 1) / (2 * 800kHz) = 2.5us (Address phase to Data phase delay).
 */
#define SPI3W_SPEED                 800000
#define SPI3W_READ_DELAY            3

/* TIMER configuration which can be modified based on requirements */
#define TIMER_NUM                   TIMER1_CH0
#define TIMER_IRQN                  TIMER1_CH0_IRQn
#define TIMER_CLOCK                 TIMER1_CLOCK

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
#define TIMER_PERIOD                (40000000)

/* RAP Configuration */
#define TIMER_EVENT_TIMEOUT         RAP_EVENT_TIMER_TIMEOUT(1, 0)

/* GPIO configuration which can be modified based on requirements */
#define OUTPUT_PIN                  P0_0
#define GPIO_OUT_PIN                GPIO_GetPinBit(OUTPUT_PIN)
#define GPIO_OUT_PORT               GPIO_GetPort(OUTPUT_PIN)
/* RAP Configuration */
#define GPIO_OUT_ACTION_TOGGLE      RAP_ACTION_GPIOA_DRTOGGLE(0)

/* Globals -------------------------------------------------------------------*/
static uint8_t spi3w_rx_buffer[32] = {0};
static volatile uint8_t fifo_len = 0;
static volatile bool is_receive_done = false;

/* Functions -----------------------------------------------------------------*/

/**
 * \brief  TIMER Interrupt Service Routine (ISR) prototype.
 */
static void TIMER_Handler(void);

/**
 * \brief  SPI3W Interrupt Service Routine (ISR) prototype.
 */
static void SPI3W_Handler(void);

/**
 * \brief  Initialize GPIO pad and pinmux settings.
 */
static void board_gpio_init(void)
{
    Pad_Config(OUTPUT_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE,
               PAD_OUT_HIGH);
    Pinmux_Config(OUTPUT_PIN, DWGPIO);
}

/**
 * \brief  Initialize GPIO peripheral.
 */
static void driver_gpio_init(void)
{
    /* Enable GPIO clock */
    RCC_ClockCmd(GPIOA_CLOCK, ENABLE);

    /* Configure GPIO parameters */
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin        = GPIO_OUT_PIN;
    GPIO_InitStruct.GPIO_Dir        = GPIO_DIR_OUT;
    GPIO_InitStruct.GPIO_INTEventEn = DISABLE;
    GPIO_Init(GPIO_OUT_PORT, &GPIO_InitStruct);
}

/**
 * \brief  Initialize SPI3W pad and pinmux settings.
 */
static void board_spi3w_init(void)
{
    Pad_Config(SPI3W_CLK_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE,
               PAD_OUT_HIGH);
    Pad_Config(SPI3W_DATA_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE,
               PAD_OUT_HIGH);
    Pad_Config(SPI3W_QB_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE,
               PAD_OUT_HIGH);

    Pinmux_Config(SPI3W_CLK_PIN, SPI3W_CLK_PINMUX);
    Pinmux_Config(SPI3W_DATA_PIN, SPI3W_DATA_PINMUX);
    Pinmux_Config(SPI3W_QB_PIN, SPI3W_QB_PINMUX);
}

/**
 * \brief  Initialize TIMER peripheral.
 */
static void driver_timer_init(void)
{
    /* Enable TIMER clock */
    RCC_ClockCmd(TIMER_CLOCK, ENABLE);

    /* Configure TIMER parameters */
    TIMER_TimeBaseInitTypeDef TIMER_InitStruct;
    TIMER_StructInit(&TIMER_InitStruct);
    TIMER_InitStruct.TIMER_ClockSrc = TIMER_CLOCK_SRC_40M;
    TIMER_InitStruct.TIMER_ClockDiv = TIMER_CLOCK_DIV_1;
    TIMER_InitStruct.PWM_En         = DISABLE;
    TIMER_InitStruct.TIMER_Mode     = TIMER_MODE_USERDEFINE;
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
 * \brief  Initialize SPI3W peripheral.
 */
static void driver_spi3w_init(void)
{
    /* Enable SPI3W clock */
    RCC_ClockCmd(SPI3W_CLOCK, ENABLE);

    /* Configure SPI3W parameters */
    SPI3W_InitTypeDef SPI3W_InitStruct;
    SPI3W_StructInit(&SPI3W_InitStruct);
    SPI3W_InitStruct.SPI3W_SysClock       = 20000000;
    SPI3W_InitStruct.SPI3W_Speed          = SPI3W_SPEED;
    SPI3W_InitStruct.SPI3W_Mode           = SPI3W_2WIRE_MODE;
    SPI3W_InitStruct.SPI3W_ReadDelay      = SPI3W_READ_DELAY;
    SPI3W_InitStruct.SPI3W_OutputDelay    = SPI3W_OE_DELAY_NONE;
    SPI3W_InitStruct.SPI3W_ExtMode        = SPI3W_NORMAL_MODE;
    SPI3W_Init(&SPI3W_InitStruct);

    /*
     * When the trigger conditions are met, the system supports the simultaneous
     * triggering of RAP events and interrupts. Users can enable the interrupt
     * function according to their actual usage needs.
     * Set to 1 to activate the interrupt function.
     */
#if 1
    /* Update vector table with ISR */
    ram_vector_table_update(SPI3W_IRQn, SPI3W_Handler);

    /* Configure NVIC */
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel         = SPI3W_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd      = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_Init(&NVIC_InitStruct);

    /* Enable SPI3W Interrupt */
    SPI3W_INTConfig(SPI3W_INT_BIT, ENABLE);
#endif

    /* Configure Quick Burst Read */
    /* Read 3 byte Data */
    SPI3W_SetQuickBurstRead(3, ENABLE);
    SPI3W_Cmd(ENABLE);
    SPI3W_SetQuickBurstPulseWidth(19);/* About 5us */
}

/**
 * \brief  Main entry.
 */
int main(void)
{
    /* Enable Global Interrupts */
    __enable_irq();

    DBG_DIRECT("Start RAP timer timeout trigger SPI3W transfer sample");

    /* Peripheral board initialization */
    board_gpio_init();
    driver_gpio_init();
    driver_timer_init();
    board_spi3w_init();
    driver_spi3w_init();

    /* Configure RAP channels */
    uint8_t channel0, channel1;
    RAP_ChannelAllocate(&channel0);
    RAP_ChannelAllocate(&channel1);

    /* Channel 0: Timer Timeout triggers SPI3W Start */
    RAP_EventRouteSet(TIMER_EVENT_TIMEOUT, channel0);
    RAP_ActionBindSet(RAP_ACTION_SPI3W_START, channel0);

    /* Channel 1: SPI3W End triggers GPIO Toggle */
    RAP_EventRouteSet(RAP_EVENT_SPI3W_END, channel1);
    RAP_ActionBindSet(GPIO_OUT_ACTION_TOGGLE, channel1);

    /* Enable RAP Mode for peripherals */
    TIMER_RAPModeCmd(TIMER_NUM, ENABLE);
    SPI3W_RAPModeCmd(ENABLE);
    GPIO_RAPModeCmd(GPIO_OUT_PORT, GPIO_OUT_PIN, ENABLE);

    /* Trigger the Timer to start the chain */
    TIMER_ActionTrigger(TIMER_NUM, TIMER_ACTION_START);

    while (1)
    {
        /*
         * Check data reception status.
         * The 'is_receive_done' flag is set in the SPI3W ISR (Transfer Complete Interrupt).
         * Transfer Complete Interrupt is triggered indicates SPI3W transfer complete.
         */
        if (is_receive_done)
        {
            /* Clear the flag immediately to acknowledge processing */
            is_receive_done = false;

            /* Print received SPI3W data */
            for (uint8_t i = 0; i < fifo_len; i++)
            {
                DBG_DIRECT("SPI3W RX Length %d, Data[%d]: 0x%X", fifo_len, i, spi3w_rx_buffer[i]);
            }

            /* Reset buffer and length for next reception */
            memset(spi3w_rx_buffer, 0, sizeof(spi3w_rx_buffer));
            fifo_len = 0;
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

        /* Add APP Code here */
        TIMER_Cmd(TIMER_NUM, ENABLE);
    }
}

/**
 * \brief  SPI3W Interrupt Service Routine (ISR).
 */
static void SPI3W_Handler(void)
{
    DBG_DIRECT("SPI3W_Handler");

    if (SPI3W_GetFlagStatus(SPI3W_FLAG_INT_IND))
    {
        /* Clear interrupt pending bit */
        SPI3W_ClearINTPendingBit(SPI3W_INT_BIT);

        fifo_len = SPI3W_GetRxDataLen();
        if (fifo_len > 0)
        {
            SPI3W_ReadBuf(spi3w_rx_buffer, fifo_len);

            /* Set flag to notify main loop that data is ready */
            is_receive_done = true;
        }
    }
}
