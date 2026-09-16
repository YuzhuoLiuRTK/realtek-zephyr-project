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
#include "rtl_adc.h"
#include "log_core.h"

/* Defines -------------------------------------------------------------------*/
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

/* ADC configuration which can be modified based on requirements */
#define ADC_PIN                         P2_0
#define ADC_IRQN                        ADC0_IRQn
#define ADC_CHANNEL                     ADC_Channel_Index_0
#define ADC_SCHEDULE                    ADC_Schedule_Index_0
/* RAP Configuration */
#define ADC_ACTION_SAMPLE               RAP_ACTION_ADC_SAMPLE
#define ADC_EVENT_DONE                  RAP_EVENT_ADC_DONE

/* GPIO configuration which can be modified based on requirements */
#define OUTPUT_PIN                      P0_0
#define GPIO_OUT_PIN                    GPIO_GetPinBit(OUTPUT_PIN)
#define GPIO_OUT_PORT                   GPIO_GetPort(OUTPUT_PIN)
/* RAP Configuration */
#define GPIO_OUT_ACTION_TOGGLE          RAP_ACTION_GPIOA_DRTOGGLE(0)

/* Functions -----------------------------------------------------------------*/
/**
 * \brief  ADC Interrupt Service Routine (ISR) prototype.
 */
void ADC_Handler(void);

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
 * \brief  Initializes pad and pinmux settings.
 */
static void board_adc_init(void)
{
    Pad_Config(ADC_PIN, PAD_SW_MODE, PAD_NOT_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE, PAD_OUT_LOW);
}

/**
 * \brief  Initializes ADC peripheral.
 */
static void driver_adc_init(void)
{
    /* Enable ADC clock */
    RCC_ClockCmd(ADC_CLOCK, ENABLE);

    /* Configure ADC parameters */
    ADC_InitTypeDef ADC_InitStruct;
    ADC_StructInit(&ADC_InitStruct);
    ADC_InitStruct.ADC_SchIndex[0]     = EXT_SINGLE_ENDED(ADC_CHANNEL);
    ADC_InitStruct.ADC_Bitmap          = BIT0; /* Enable Schedule Index 0 */
    ADC_InitStruct.ADC_PowerAlwaysOnEn = ENABLE;
    ADC_Init(ADC, &ADC_InitStruct);

    /*
     * When the trigger conditions are met, the system supports the simultaneous
     * triggering of RAP events and interrupts. Users can enable the interrupt
     * function according to their actual usage needs.
     * Set to 1 to activate the interrupt function.
     */
#if 0
    /* Update vector table with ISR */
    ram_vector_table_update(ADC_IRQN, ADC_Handler);

    /* Configure NVIC */
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel         = ADC_IRQN;
    NVIC_InitStruct.NVIC_IRQChannelCmd      = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_Init(&NVIC_InitStruct);

    /* Enable ADC One Shot Done Interrupt */
    ADC_INTConfig(ADC, ADC_INT_ONE_SHOT_DONE, ENABLE);
#endif
}

/**
  * \brief  Main entry.
  */
int main(void)
{
    /* Enable Global Interrupts */
    __enable_irq();

    DBG_DIRECT("Start RAP timer timeout trigger ADC oneshot sample sample");

    /* Peripheral initialization */
    board_gpio_init();
    driver_gpio_init();
    driver_timer_init();
    board_adc_init();
    driver_adc_init();

    /* Configure RAP channel */
    uint8_t channel0, channel1;
    RAP_ChannelAllocate(&channel0);
    RAP_ChannelAllocate(&channel1);

    /* Route TIMER Timeout Event to RAP channel0 */
    RAP_EventRouteSet(TIMER_EVENT_TIMEOUT, channel0);
    /* Bind ADC One Shot Sample Action to RAP channel0 */
    RAP_ActionBindSet(ADC_ACTION_SAMPLE, channel0);

    /* Route ADC Done Event to RAP channel1 */
    RAP_EventRouteSet(ADC_EVENT_DONE, channel1);
    /* Bind GPIO Toggle Action to RAP channel1 */
    RAP_ActionBindSet(GPIO_OUT_ACTION_TOGGLE, channel1);

    /* Enable RAP Mode for Peripherals */
    TIMER_RAPModeCmd(TIMER_NUM, ENABLE);
    ADC_RAPModeCmd(ADC, ENABLE);
    GPIO_RAPModeCmd(GPIO_OUT_PORT, GPIO_OUT_PIN, ENABLE);

    /* Start TIMER */
    TIMER_ActionTrigger(TIMER_NUM, TIMER_ACTION_START);

    while (1)
    {
    }

    return 0;
}

/**
 * \brief  ADC Interrupt Service Routine (ISR).
 */
void ADC_Handler(void)
{
    DBG_DIRECT("ADC_Handler");

    if (ADC_GetINTStatus(ADC, ADC_INT_ONE_SHOT_DONE) == SET)
    {
        ADC_ClearINTPendingBit(ADC, ADC_INT_ONE_SHOT_DONE);

        uint32_t sample_data = ADC_ReadRawData(ADC, ADC_SCHEDULE);
        DBG_DIRECT("ADC_INT_ONE_SHOT_DONE -> sample_data: %d", sample_data);
    }
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
