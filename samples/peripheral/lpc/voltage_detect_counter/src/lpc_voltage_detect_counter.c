/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: LicenseRef-Realtek-5-Clause
 */

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include "rtl_lpc.h"
#include "rtl_nvic.h"
#include "rtl_pinmux.h"
#include "rtl_rcc.h"
#include "log_core.h"

/* Defines -------------------------------------------------------------------*/
/* LPC configuration which can be modified based on requirements */
#define LPC_CAPTURE_PIN                 P2_2
#define LPC_CAPTURE_CHANNEL             LPC_CHANNEL_ADC2

/*
 * Configure LPC Threshold Voltage.
 * Range depends on specific IC (e.g., LPC_1080_mV means 1.08V).
 */
#define LPC_COMPARE_VOLTAGE             LPC_1080_mV

/*
 * Configure Trigger Condition.
 * LPC_VIN_OVER_VTH: Trigger when Input Voltage > Threshold.
 * LPC_VIN_UNDER_VTH: Trigger when Input Voltage < Threshold.
 */
#define LPC_VOLTAGE_DETECT_EDGE         LPC_VIN_OVER_VTH

/*
 * Change LPC Compare Counter
 * Interrupt will trigger when the counter reaches this value.
 */
#define LPC_COMPARE_COUNTER             10

/* Functions -----------------------------------------------------------------*/
/**
 * \brief  LPC Interrupt Service Routine (ISR) prototype.
 */
void LPC_Handler(void);

/**
 * \brief  Initializes pad settings and pinmux settings.
 */
void board_lpc_init(void)
{
    /*
     * Set LPC capture pin to Software Mode.
     * Output is disabled, Pull is none (High-Z) for accurate voltage measurement.
     * Set default output to Low for safety.
     */
    Pad_Config(LPC_CAPTURE_PIN, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE,
               PAD_OUT_HIGH);

    /* Set Pinmux to Idle/Analog mode */
    Pinmux_Config(LPC_CAPTURE_PIN, IDLE_MODE);
}

/**
 * \brief  Initializes LPC peripheral.
 */
void driver_lpc_init(void)
{
    /* Reset LPC to default state */
    LPC_DeInit(LPC0);

    /* Enable LPC Clock */
    RCC_ClockCmd(LPC_CLOCK, ENABLE);

    /* Initialize LPC Parameters */
    LPC_InitTypeDef LPC_InitStruct;
    LPC_StructInit(&LPC_InitStruct);
    LPC_InitStruct.LPC_Channel    = LPC_CAPTURE_CHANNEL;
    LPC_InitStruct.LPC_Edge       = LPC_VOLTAGE_DETECT_EDGE;
    LPC_InitStruct.LPC_Threshold  = LPC_COMPARE_VOLTAGE;
    LPC_Init(LPC0, &LPC_InitStruct);

    /* Update vector table with ISR */
    ram_vector_table_update(LPC_IRQn, LPC_Handler);

    /* Configure NVIC */
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = LPC_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 2;
    NVIC_Init(&NVIC_InitStruct);

    /* Configure LPC Counter */
    LPC_CounterReset(LPC0);
    LPC_SetComparator(LPC0, LPC_COMPARE_COUNTER);

    /* Enable Counter Compare Interrupt */
    LPC_INTConfig(LPC0, LPC_INT_COUNTER_COMPARE, ENABLE);

    /* Enable the Counter function */
    LPC_CounterCmd(LPC0, ENABLE);

}

/**
 * \brief  Main entry.
 */
int main(void)
{
    /* Enable Global Interrupts */
    __enable_irq();

    DBG_DIRECT("Start lpc counter sample");

    /* Peripheral initialization */
    board_lpc_init();
    driver_lpc_init();

    while (1)
    {
    }
}

/**
 * \brief  LPC Interrupt Service Routine (ISR).
 */
void LPC_Handler(void)
{
    /* Check if Counter Compare Interrupt occurred */
    if (LPC_GetINTStatus(LPC0, LPC_INT_COUNTER_COMPARE) == SET)
    {
        DBG_DIRECT("LPC_Handler: Counter Reached Threshold: %d", LPC_COMPARE_COUNTER);

        /* Disable the interrupt to prevent continuous triggering if the voltage */
        LPC_INTConfig(LPC0, LPC_INT_COUNTER_COMPARE, DISABLE);

        /* Clear the interrupt status */
        LPC_ClearINTStatus(LPC0, LPC_INT_COUNTER_COMPARE);
    }
}
