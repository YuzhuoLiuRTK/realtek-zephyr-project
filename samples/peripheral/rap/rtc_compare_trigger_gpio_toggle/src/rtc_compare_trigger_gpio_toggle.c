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

/* RTC Reload Mode Configuration
 * Select the mode used to reload the comparator value when RTC compares.
 *
 * - SAMPLE_CONFIG_RTC_USING_RELOAD_BY_AUTO_RELOAD:
 *    The RTC hardware automatically loads the preset reload value into the
 *    compare register immediately when RTC compares.
 *
 * - SAMPLE_CONFIG_RTC_USING_RELOAD_BY_ACTION_RELOAD:
 *    The RTC Compare Event is routed to the RAP to trigger the specific
 *    Reload Action (RAP_ACTION_RTC_RELOAD_COMP) to execute the manual reload.
 *
 * - SAMPLE_CONFIG_RTC_USING_RELOAD_BY_SHORTCUT_RELOAD:
 *    The RTC Compare Event directly triggers the internal Reload Task (Shortcut)
 *    without CPU or RAP intervention to execute the manual reload.
 */
#define SAMPLE_CONFIG_RTC_USING_RELOAD_BY_AUTO_RELOAD         1
#define SAMPLE_CONFIG_RTC_USING_RELOAD_BY_ACTION_RELOAD       0
#define SAMPLE_CONFIG_RTC_USING_RELOAD_BY_SHORTCUT_RELOAD     0

#if ((SAMPLE_CONFIG_RTC_USING_RELOAD_BY_AUTO_RELOAD + \
      SAMPLE_CONFIG_RTC_USING_RELOAD_BY_ACTION_RELOAD + \
      SAMPLE_CONFIG_RTC_USING_RELOAD_BY_SHORTCUT_RELOAD) >= 2)
#error "SAMPLE CONFIG ERROR!!!"
#endif

/* RTC configuration which can be modified based on requirements */
/*
 * Configure RTC Prescaler parameters.
 *
 * Calculation formula:
 *  - Tick_Freq = Clock_Src(32kHz) / (PSC + 1).
 *
 * Based on the following settings:
 *  - RTC Clock Source: 32kHz.
 *
 * Define the RTC_PSC_VALUE as (3200 - 1) which generates a 10kHz(100ms) tick.
 */
#define RTC_PSC_VALUE                   (3200 - 1)

/*
 * Configure RTC Compare and Reload parameters.
 *
 * Calculation formula:
 *  - Timeout = Value * (1 / Tick_Freq).
 *
 * Based on the following settings:
 *  - RTC Tick Frequency: 10Hz (from Prescaler settings).
 *
 * Define RTC_COMP_VALUE as 20 which timeout is 2 seconds.
 * Define RTC_COMP_RELOAD_VALUE as 10 which reload time is 1 seconds.
 */
#define RTC_COMP_NUM                    RTC_COMP0
#define RTC_COMP_INT                    RTC_INT_COMP0
#define RTC_COMP_VALUE                  (20)
#define RTC_COMP_RELOAD_VALUE           (10)

/* RAP Configuration */
#define RTC_EVENT_COMPARE               RAP_EVENT_RTC_COMPARE(0)
#define RTC_ACTION_RELOAD               RAP_ACTION_RTC_RELOAD_COMP(0)
#define RTC_SHORTCUT_EVENT_COMPARE      RTC_EVENT_COMP0
#define RTC_SHORTCUT_ACTION_RELOAD      RTC_ACTION_RELOAD_COMP0

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

    /* Configure RTC prescaler to generate the tick */
    RTC_SetPrescaler(RTC_PSC_VALUE);

    /* Configure RTC compare and reload value */
    RTC_SetCompValue(RTC_COMP_NUM, RTC_COMP_VALUE);
    RTC_SetCompReloadValue(RTC_COMP_NUM, RTC_COMP_RELOAD_VALUE);

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

    /* Enable RTC interrupt */
    RTC_NVICCmd(ENABLE);
    RTC_INTConfig(RTC_COMP_INT, ENABLE);
#endif
}

/**
  * \brief  Main entry.
  */
int main(void)
{
    /* Enable Global Interrupts */
    __enable_irq();

    DBG_DIRECT("Start RAP rtc compare trigger gpio toggle sample");

    /* Peripheral initialization */
    board_gpio_init();
    driver_gpio_init();
    driver_rtc_init();

    /* Configure RAP channel */
    uint8_t channel0;
    RAP_ChannelAllocate(&channel0);

    /* Route RTC Compare event to RAP channel */
    RAP_EventRouteSet(RTC_EVENT_COMPARE, channel0);
    /* Bind GPIO Toggle action to RAP channel */
    RAP_ActionBindSet(GPIO_OUT_ACTION_TOGGLE, channel0);

    /*
     * Configure RTC Comparator Reload Mechanism.
     * Select one of the three methods to reload the comparator value after a timeout.
     */
#if (SAMPLE_CONFIG_RTC_USING_RELOAD_BY_AUTO_RELOAD == 1)
    /* RTC automatically loads RELOAD_VALUE immediately after compare match */
    extern void RTC_CompAutoReloadCmd(RTCCompIndex_TypeDef Index, FunctionalState NewState);
    RTC_CompAutoReloadCmd(RTC_COMP_NUM, ENABLE);

#elif (SAMPLE_CONFIG_RTC_USING_RELOAD_BY_ACTION_RELOAD == 1)
    /* Bind the Reload Action to the specific RAP channel to trigger reload */
    RAP_ActionBindSet(RTC_ACTION_RELOAD, channel0);

#elif (SAMPLE_CONFIG_RTC_USING_RELOAD_BY_SHORTCUT_RELOAD == 1)
    /* Directly trigger Reload Task (Action) when compare event occurs by shortcut */
    RTC_ShortcutCmd(RTC_SHORTCUT_ACTION_RELOAD, RTC_SHORTCUT_EVENT_COMPARE, ENABLE);
#endif

    /* Enable RTC RAP mode and GPIO RAP mode */
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
    if (RTC_GetINTStatus(RTC_COMP_INT) == SET)
    {
        /*
         * User code can be added here. For example: Print Info.
         * Note: Using "DBG_DIRECT" to printf info in an ISR takes a long time
         * and is not recommended. It is used here solely for demonstration sample.
         */
        DBG_DIRECT("RTC_Handler: RTC_INT_COMP%d", RTC_COMP_NUM);

        /* Clear interrupt status */
        RTC_ClearCompINT(RTC_COMP_NUM);
    }
    DBG_DIRECT("RTC_Handler: RTC Current Counter: %d", RTC_GetCounter());
}
