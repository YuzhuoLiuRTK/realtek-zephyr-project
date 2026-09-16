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
#include "rtl_timer.h"
#include "rtl_rap.h"
#include "log_core.h"

/* Defines -------------------------------------------------------------------*/

/* TIMER One Shot Mode Configuration */
/*
 * Select the mechanism used to stop the timer after a timeout event.
 *
 * - SAMPLE_CONFIG_TIMER_USING_ONE_SHOT_BY_ACTION_STOP:
 *   The TIMER Timeout Event is routed to the RAP to trigger the specific
 *   Stop Action (RAP_ACTION_TIMER_STOP) to stop the timer.
 *
 * - SAMPLE_CONFIG_TIMER_USING_ONE_SHOT_BY_SHORTCUT_STOP:
 *   The TIMER Timeout Event directly triggers the internal Stop Task (Shortcut)
 *   without CPU or RAP intervention to stop the timer.
 */
#define SAMPLE_CONFIG_TIMER_USING_ONE_SHOT_BY_ACTION_STOP     0
#define SAMPLE_CONFIG_TIMER_USING_ONE_SHOT_BY_SHORTCUT_STOP   0

#if ((SAMPLE_CONFIG_TIMER_USING_ONE_SHOT_BY_ACTION_STOP + \
      SAMPLE_CONFIG_TIMER_USING_ONE_SHOT_BY_SHORTCUT_STOP) >= 2)
#error "SAMPLE CONFIG ERROR!!!"
#endif

/* TIMER configuration which can be modified based on requirements */
#define TIMER_NUM                       TIMER1_CH0
#define TIMER_IRQN                      TIMER1_CH0_IRQn

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
#define TIMER_PERIOD                    (40000000)

/* RAP Configuration */
#define TIMER_EVENT_TIMEOUT             RAP_EVENT_TIMER_TIMEOUT(1, 0)
#define TIMER_ACTION_STOP               RAP_ACTION_TIMER_STOP(1, 0)

/* GPIO configuration which can be modified based on requirements */
#define OUTPUT_PIN                      P0_0
#define GPIO_OUT_PIN                    GPIO_GetPinBit(OUTPUT_PIN)
#define GPIO_OUT_PORT                   GPIO_GetPort(OUTPUT_PIN)
/* RAP Configuration */
#define GPIO_OUT_ACTION_TOGGLE          RAP_ACTION_GPIOA_DRTOGGLE(0)

/* Functions -----------------------------------------------------------------*/
/**
 * \brief  TIMER Interrupt Service Routine (ISR) prototype.
 */
void TIMER_Handler(void);

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
 * \brief  Initializes TIMER peripheral.
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
  * \brief  Main entry.
  */
int main(void)
{
    /* Enable Global Interrupts */
    __enable_irq();

    DBG_DIRECT("Start RAP timer timeout trigger gpio toggle sample");

    /* Peripheral initialization */
    board_gpio_init();
    driver_gpio_init();
    driver_timer_init();

    /* Configure RAP channel */
    uint8_t channel0;
    RAP_ChannelAllocate(&channel0);

    /* Route TIMER Timeout Event to RAP channel */
    RAP_EventRouteSet(TIMER_EVENT_TIMEOUT, channel0);
    /* Bind GPIO Toggle Action to RAP channel */
    RAP_ActionBindSet(GPIO_OUT_ACTION_TOGGLE, channel0);

    /* Configure TIMER One Shot Mode */
#if (SAMPLE_CONFIG_TIMER_USING_ONE_SHOT_BY_SHORTCUT_STOP == 1)
    /* Directly trigger Stop Task (Action) when timeout event occurs by shortcut */
    TIMER_ShortcutCmd(TIMER_NUM, TIMER_SHORTCUT_ACTION, TIMER_SHORTCUT_EVENT, ENABLE);

#elif (SAMPLE_CONFIG_TIMER_USING_ONE_SHOT_BY_ACTION_STOP == 1)
    /* Bind the Stop Action to the specific RAP channel to trigger stop */
    RAP_ActionBindSet(TIMER_ACTION_STOP, channel0);
#endif

    /* Enable RAP Mode for TIMER and GPIO */
    TIMER_RAPModeCmd(TIMER_NUM, ENABLE);
    GPIO_RAPModeCmd(GPIO_OUT_PORT, GPIO_OUT_PIN, ENABLE);

    /* Start TIMER */
    TIMER_ActionTrigger(TIMER_NUM, TIMER_ACTION_START);

    while (1)
    {
    }

    return 0;
}

/**
 * \brief  TIMER Interrupt Service Routine (ISR).
 */
void TIMER_Handler(void)
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
