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
#include "utils.h"

/* Defines -------------------------------------------------------------------*/
/* GPIO configuration which can be modified based on requirements */
#define OUTPUT_PIN                      P0_0
#define GPIO_OUT_PIN                    GPIO_GetPinBit(OUTPUT_PIN)
#define GPIO_OUT_PORT                   GPIO_GetPort(OUTPUT_PIN)
/* RAP Configuration */
#define GPIO_OUT_ACTION_TOGGLE          RAP_ACTION_GPIOA_DRTOGGLE(0)

#define INPUT_PIN                       P0_1
#define GPIO_IN_PIN                     GPIO_GetPinBit(INPUT_PIN)
#define GPIO_IN_PORT                    GPIO_GetPort(INPUT_PIN)
#define GPIO_IN_IRQN                    GPIOA1_IRQn
/* RAP Configuration */
#define GPIO_IN_EVENT_IN                RAP_EVENT_GPIOA(1)

/* Functions -----------------------------------------------------------------*/
/**
 * \brief  GPIO Interrupt Service Routine (ISR) prototype.
 */
void GPIO_Pin_Handler(void);

/**
 * \brief  Initializes pad and pinmux settings.
 */
static void board_gpio_init(void)
{
    Pad_Config(INPUT_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_HIGH);
    Pinmux_Config(INPUT_PIN, DWGPIO);

    Pad_Config(OUTPUT_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE, PAD_OUT_HIGH);
    Pinmux_Config(OUTPUT_PIN, DWGPIO);
}

/**
 * \brief  Initializes GPIO peripheral and Interrupts.
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

    /* Configure GPIO parameters as input mode */
    GPIO_InitStruct.GPIO_Pin        = GPIO_IN_PIN;
    GPIO_InitStruct.GPIO_Dir        = GPIO_DIR_IN;
    GPIO_InitStruct.GPIO_INTEventEn = ENABLE;
    GPIO_InitStruct.GPIO_Trigger    = GPIO_TRIGGER_EDGE;
    GPIO_InitStruct.GPIO_Polarity   = GPIO_POLARITY_ACTIVE_LOW;

    /*
     * Configure GPIO Debounce parameters.
     * Calculation formula: debounce time = (CntLimit + 1) * DEB_CLK
     * Enable GPIO doubonce, debounce time is set as 1ms in this settings(32/32000 = 1ms).
     */
    GPIO_InitStruct.GPIO_DebounceEn    = ENABLE;
    GPIO_InitStruct.GPIO_DebClockSrc   = GPIO_DEFAULT_DEB_CLOCK_SRC;
    GPIO_InitStruct.GPIO_DebClockDiv   = GPIO_DEB_CLOCK_DIV_1;
    GPIO_InitStruct.GPIO_DebCountLimit = 32 - 1;

    GPIO_Init(GPIO_IN_PORT, &GPIO_InitStruct);

    /* When the trigger conditions are met, the system supports the simultaneous
     * triggering of RAP events and interrupts. Users can enable the interrupt
     * function according to their actual usage needs.
     * Set to 1 to activate the interrupt function.
     */
#if 0
    /* Update vector table with ISR */
    ram_vector_table_update(GPIO_IN_IRQN, GPIO_Pin_Handler);

    /* Configure NVIC */
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel         = GPIO_IN_IRQN;
    NVIC_InitStruct.NVIC_IRQChannelCmd      = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_Init(&NVIC_InitStruct);

    /* Enable interrupt */
    GPIO_MaskINTConfig(GPIO_IN_PORT, GPIO_IN_PIN, ENABLE);
    GPIO_INTConfig(GPIO_IN_PORT, GPIO_IN_PIN, ENABLE);

    /*
     * Delay 2T debounce time to stabilize if debounce function is enabled.
     * Note that debounce time is set as 1ms in this sample.
     */
    if (GPIO_InitStruct.GPIO_DebounceEn == ENABLE)
    {
        platform_delay_ms(2);
    }

    GPIO_ClearINTPendingBit(GPIO_IN_PORT, GPIO_IN_PIN);
    GPIO_MaskINTConfig(GPIO_IN_PORT, GPIO_IN_PIN, DISABLE);
#endif
}

/**
 * \brief  Generates square wave pulses on the specified pin.
 * \param  Pin_Num: The Pin_Num to control.
 * \param  pulse_count: Number of pulses to generate.
 *
 * Output waveform:
 *  _____       _____
 * |     |     |     |
 * |     |_____|     |_____
 *  20ms  20ms
 */
static void pad_generate_pulse(uint8_t Pin_Num, uint8_t pulse_count)
{
    for (volatile uint8_t i = 0; i < pulse_count; i++)
    {
        Pad_Config(Pin_Num, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE, PAD_OUT_HIGH);
        platform_delay_ms(20);
        Pad_Config(Pin_Num, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_ENABLE, PAD_OUT_LOW);
        platform_delay_ms(20);
    }
}

/**
  * \brief  Main entry.
  */
int main(void)
{
    /* Enable Global Interrupts */
    __enable_irq();

    DBG_DIRECT("Start RAP gpio in trigger gpio toggle sample");

    /* Peripheral initialization */
    board_gpio_init();
    driver_gpio_init();

    /* Configure RAP channel */
    uint8_t channel0;
    RAP_ChannelAllocate(&channel0);

    /* Route GPIO IN Event to RAP channel */
    RAP_EventRouteSet(GPIO_IN_EVENT_IN, channel0);
    /* Bind GPIO Toggle Action to RAP channel */
    RAP_ActionBindSet(GPIO_OUT_ACTION_TOGGLE, channel0);

    /* Enable GPIO RAP Mode */
    GPIO_RAPModeCmd(GPIO_OUT_PORT, GPIO_OUT_PIN, ENABLE);
    GPIO_RAPModeCmd(GPIO_IN_PORT, GPIO_IN_PIN, ENABLE);

    /*
     * Simulate pulse input using P0_2 (Users can short P0_2 to INPUT_PIN).
     * When INPUT_PIN detects the falling-edge pulse, a RAP event will be triggered.
     */
    pad_generate_pulse(P0_2, 2);

    while (1)
    {
    }

    return 0;
}

/**
 * \brief  GPIO Interrupt Service Routine (ISR).
 */
void GPIO_Pin_Handler(void)
{
    /* Mask and disable interrupt */
    GPIO_INTConfig(GPIO_IN_PORT, GPIO_IN_PIN, DISABLE);
    GPIO_MaskINTConfig(GPIO_IN_PORT, GPIO_IN_PIN, ENABLE);

    /*
     * User code can be added here. For example: Print Info.
     * Note: Using "DBG_DIRECT" to printf info in an ISR takes a long time
     * and is not recommended. It is used here solely for demonstration sample.
     */
    DBG_DIRECT("Enter GPIO_Pin_Handler success");

    /* Clear interrupt status */
    GPIO_ClearINTPendingBit(GPIO_IN_PORT, GPIO_IN_PIN);

    /* Unmask and re-enable interrupt */
    GPIO_MaskINTConfig(GPIO_IN_PORT, GPIO_IN_PIN, DISABLE);
    GPIO_INTConfig(GPIO_IN_PORT, GPIO_IN_PIN, ENABLE);
}
