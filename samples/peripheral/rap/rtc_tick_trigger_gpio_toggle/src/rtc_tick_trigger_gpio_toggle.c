/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: LicenseRef-Realtek-5-Clause
 */

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include "rtl_rcc.h"
#include "rtl_nvic.h"
#include "rtl_pinmux.h"
#include "rtl_gpio.h"
#include "rtl_rap.h"
#include "rtl_rtc.h"
#include "log_core.h"

/* Defines -------------------------------------------------------------------*/

/* RTC configuration which can be modified based on requirements */
/*
 * Configure RTC Prescaler parameters.
 *
 * Calculation formula:
 *  - Tick_Freq = Clock_Src / (PSC + 1).
 *
 * Based on the following settings:
 *  - RTC Clock Source: 32kHz.
 *
 * Define the RTC_PSC_VALUE as (3200 - 1) which generates a 100ms time base.
 */
#define RTC_PSC_VALUE                   (3200 - 1)

/* RAP Configuration */
#define RTC_EVENT_TICK                  RAP_EVENT_RTC_TICK

/* GPIO configuration which can be modified based on requirements */
#define OUTPUT_PIN                      P0_0
#define GPIO_OUT_PIN                    GPIO_GetPinBit(OUTPUT_PIN)
#define GPIO_OUT_PORT                   GPIO_GetPort(OUTPUT_PIN)
/* RAP Configuration */
#define GPIO_OUT_ACTION_TOGGLE          RAP_ACTION_GPIOA_DRTOGGLE(0)

/* Functions -----------------------------------------------------------------*/
/**
 * \brief  RTC Interrupt Service Routine (ISR) prototype.
 */
void RTC_Handler(void);

/**
 * \brief  Initializes pad and pinmux settings.
 */
static void board_gpio_init(void)
{
    Pad_Config(OUTPUT_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE, PAD_OUT_HIGH);

    Pinmux_Config(OUTPUT_PIN, DWGPIO);
}

/**
 * \brief  Initializes GPIO peripheral.
 */
static void driver_gpio_init(void)
{
    /* Enable GPIO clock */
    RCC_ClockCmd(GPIOA_CLOCK, ENABLE);

    /* Configure GPIO parameters as output mode */
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin        = GPIO_OUT_PIN;
    GPIO_InitStruct.GPIO_Dir        = GPIO_DIR_OUT;
    GPIO_InitStruct.GPIO_INTEventEn = DISABLE;
    GPIO_Init(GPIO_OUT_PORT, &GPIO_InitStruct);
}

/**
 * \brief  Initializes RTC peripheral and Interrupts.
 */
static void driver_rtc_init(void)
{
    /* Enable RTC clock */
    RCC_ClockCmd(RTC_CLOCK, ENABLE);
    RTC_DeInit();

    /* Configure RTC prescaler to generate the Tick frequency */
    RTC_SetPrescaler(RTC_PSC_VALUE);

    /* Reset the RTC counter to ensure the initial timing starts from zero */
    RTC_ResetCounter();

    /*
     * When the trigger conditions are met, the system supports the simultaneous
     * triggering of RAP events and interrupts. Users can enable the interrupt
     * function according to their actual usage needs.
     * Set to 1 to activate the interrupt function.
     */
#if 0
    /* Update vector table with ISR */
    ram_vector_table_update(RTC_IRQn, RTC_Handler);

    /* Configure NVIC */
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel         = RTC_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd      = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_Init(&NVIC_InitStruct);

    /* Enable RTC Interrupt */
    RTC_NVICCmd(ENABLE);
    RTC_INTConfig(RTC_INT_TICK, ENABLE);
#endif
}

/**
  * \brief  Main entry.
  */
int main(void)
{
    /* Enable Global Interrupts */
    __enable_irq();

    DBG_DIRECT("Start RAP rtc tick trigger gpio toggle sample");

    /* Peripheral initialization */
    board_gpio_init();
    driver_gpio_init();
    driver_rtc_init();

    /* Configure RAP Channel */
    uint8_t channel0;
    RAP_ChannelAllocate(&channel0);

    /* Route RTC Tick Event to RAP channel */
    RAP_EventRouteSet(RTC_EVENT_TICK, channel0);
    /* Bind GPIO Toggle Action to RAP channel */
    RAP_ActionBindSet(GPIO_OUT_ACTION_TOGGLE, channel0);

    /* Enable RAP Mode for RTC and GPIO */
    RTC_RAPModeCmd(ENABLE);
    GPIO_RAPModeCmd(GPIO_OUT_PORT, GPIO_OUT_PIN, ENABLE);

    /* Start RTC */
    RTC_ActionTrigger(RTC_ACTION_START);

    while (1)
    {
    }

    return 0;
}

/**
 * \brief  RTC Interrupt Service Routine (ISR).
 */
void RTC_Handler(void)
{
    if (RTC_GetINTStatus(RTC_INT_TICK) == SET)
    {
        /*
         * User code can be added here. For example: Print Info.
         * Note: Using "DBG_DIRECT" to printf info in an ISR takes a long time
         * and is not recommended. It is used here solely for demonstration sample.
         */
        DBG_DIRECT("RTC_Handler: RTC_INT_TICK");

        /* Clear interrupt status */
        RTC_ClearTickINT();
    }
    DBG_DIRECT("RTC_Handler: RTC Current Counter: %d", RTC_GetCounter());
}
