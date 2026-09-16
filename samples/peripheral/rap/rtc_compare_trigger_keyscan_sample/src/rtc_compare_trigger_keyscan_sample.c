/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: LicenseRef-Realtek-5-Clause
 */

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <string.h>
#include "rtl_rcc.h"
#include "rtl_nvic.h"
#include "rtl_pinmux.h"
#include "rtl_gpio.h"
#include "rtl_rtc.h"
#include "rtl_rap.h"
#include "rtl_keyscan.h"
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
 * Define the RTC_PSC_VALUE as (320 - 1) which generates a 100Hz(10ms) tick.
 */
#define RTC_PSC_VALUE                   (320 - 1)

/*
 * Configure RTC Compare and Reload parameters.
 *
 * Calculation formula:
 *  - Timeout = Value * (1 / Tick_Freq).
 *
 * Based on the following settings:
 *  - RTC Tick Frequency: 100Hz (from Prescaler settings).
 *
 * Define RTC_COMP_VALUE as 5 which timeout is 50ms.
 * Define RTC_COMP_RELOAD_VALUE as 5 which reload time is 50ms.
 */
#define RTC_COMP_NUM                    RTC_COMP0
#define RTC_COMP_INT                    RTC_INT_COMP0
#define RTC_COMP_VALUE                  (5)
#define RTC_COMP_RELOAD_VALUE           (5)

/* RAP Configuration */
#define RTC_EVENT_COMPARE               RAP_EVENT_RTC_COMPARE(0)
#define RTC_ACTION_RELOAD               RAP_ACTION_RTC_RELOAD_COMP(0)

/* KEYSCAN configuration which can be modified based on requirements */
/* KEYSCAN row and column size */
#define KEYBOARD_ROW_SIZE               2
#define KEYBOARD_COLUMN_SIZE            2

/* KEYSCAN pin definitions */
#define KEYBOARD_ROW_0                  P4_0
#define KEYBOARD_ROW_1                  P4_1
#define KEYBOARD_COLUMN_0               P4_2
#define KEYBOARD_COLUMN_1               P4_3

/* Globals -------------------------------------------------------------------*/
static struct
{
    uint16_t length;
    struct
    {
        uint16_t column: 5;   /**< KEYSCAN column buffer data */
        uint16_t row: 5;      /**< KEYSCAN row buffer data */
    } key[100];
} keyscan_current_data;

volatile bool is_first_pressed = false;

/* Functions -----------------------------------------------------------------*/
/**
 * \brief  RTC Interrupt Service Routine (ISR) prototype.
 */
void RTC_Handler(void);

/**
 * \brief  KEYSCAN Interrupt Service Routine (ISR) prototype.
 */
void KEYSCAN_Handler(void);

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
 * \brief  Initializes pad and pinmux settings.
 */
static void board_keyscan_init(void)
{
    Pad_Config(KEYBOARD_ROW_0, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE,
               PAD_OUT_LOW);
    Pad_Config(KEYBOARD_ROW_1, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE,
               PAD_OUT_LOW);
    Pad_Config(KEYBOARD_COLUMN_0, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE,
               PAD_OUT_LOW);
    Pad_Config(KEYBOARD_COLUMN_1, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE,
               PAD_OUT_LOW);

    Pinmux_Config(KEYBOARD_ROW_0, KEY_ROW_0);
    Pinmux_Config(KEYBOARD_ROW_1, KEY_ROW_1);
    Pinmux_Config(KEYBOARD_COLUMN_0, KEY_COL_0);
    Pinmux_Config(KEYBOARD_COLUMN_1, KEY_COL_1);
}

/**
 * \brief  Initializes KEYSCAN peripheral and Interrupts.
 * \param  Manual_Sel: Selects the hardware trigger source (Key Press or Register Bit).
 */
static void driver_keyscan_init(KEYSCANManualSel_TypeDef Manual_Sel)
{
    /* Enable KEYSCAN clock */
    RCC_ClockCmd(KEYSCAN_CLOCK, ENABLE);

    /* Configure KEYSCAN parameters */
    KEYSCAN_InitTypeDef KEYSCAN_InitStruct;
    KEYSCAN_StructInit(&KEYSCAN_InitStruct);
    KEYSCAN_InitStruct.KEYSCAN_RowSize    = KEYBOARD_ROW_SIZE;
    KEYSCAN_InitStruct.KEYSCAN_ColSize    = KEYBOARD_COLUMN_SIZE;
    KEYSCAN_InitStruct.KEYSCAN_ScanMode   = KEYSCAN_MANUAL_SCAN_MODE;
    KEYSCAN_InitStruct.KEYSCAN_DetectMode = KEYSCAN_DETECT_MODE_EDGE;
    KEYSCAN_InitStruct.KEYSCAN_ManualSel  = Manual_Sel;
    KEYSCAN_Init(KEYSCAN, &KEYSCAN_InitStruct);

    /* Update vector table with ISR */
    ram_vector_table_update(KEYSCAN_IRQn, KEYSCAN_Handler);

    /* Configure NVIC */
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel         = KEYSCAN_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd      = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_Init(&NVIC_InitStruct);

    /* Enable KEYSCAN interrupt */
    KEYSCAN_INTConfig(KEYSCAN, KEYSCAN_INT_SCAN_END, ENABLE);
    KEYSCAN_ClearINTPendingBit(KEYSCAN, KEYSCAN_INT_SCAN_END);
    KEYSCAN_INTMask(KEYSCAN, KEYSCAN_INT_SCAN_END, DISABLE);
}


/**
  * \brief  Main entry.
  */
int main(void)
{
    /* Enable Global Interrupts */
    __enable_irq();

    DBG_DIRECT("Start RAP rtc compare trigger KEYSCAN sample");

    /* Initialize global data */
    keyscan_current_data.length = 0;
    memset(keyscan_current_data.key, 0, sizeof(keyscan_current_data.key));

    /* Peripheral initialization */
    board_keyscan_init();
    driver_keyscan_init(KEYSCAN_MANUAL_SEL_KEY);
    driver_rtc_init();

    /* Configure RAP channel */
    uint8_t channel0;
    RAP_ChannelAllocate(&channel0);

    /* Route RTC Compare event to RAP channel */
    RAP_EventRouteSet(RTC_EVENT_COMPARE, channel0);
    /* Bind KEYSCAN Manual Action to RAP channel */
    RAP_ActionBindSet(RAP_ACTION_KEYSCAN_MANUAL, channel0);
    /* Bind RTC Reload Action to RAP channel */
    RAP_ActionBindSet(RTC_ACTION_RELOAD, channel0);

    /* Enable KEYSCAN */
    KEYSCAN_Cmd(KEYSCAN, ENABLE);

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

/**
 * \brief  KEYSCAN Interrupt Service Routine (ISR).
 */
void KEYSCAN_Handler(void)
{
    /* Check Scan End Interrupt */
    if (KEYSCAN_GetFlagState(KEYSCAN, KEYSCAN_INT_FLAG_SCAN_END) == SET)
    {
        /* Mask interrupt and reset data structure */
        KEYSCAN_INTMask(KEYSCAN, KEYSCAN_INT_SCAN_END, ENABLE);
        keyscan_current_data.length = 0;
        memset(keyscan_current_data.key, 0, sizeof(keyscan_current_data.key));

        /* If KEYSCAN FIFO is not empty, keys are currently pressed. */
        if (KEYSCAN_GetFlagState(KEYSCAN, KEYSCAN_FLAG_EMPTY) != SET)
        {
            /* Read KEYSCAN FIFO data */
            uint16_t fifo_length = KEYSCAN_GetFIFODataNum(KEYSCAN);
            KEYSCAN_Read(KEYSCAN, (uint16_t *)&keyscan_current_data.key[0], fifo_length);
            keyscan_current_data.length = fifo_length;

            if (fifo_length == 1)
            {
                DBG_DIRECT("KEYSCAN One Key Press Detected: (%d, %d)",
                           keyscan_current_data.key[0].row, keyscan_current_data.key[0].column);
            }
            else if (fifo_length == 2)
            {
                DBG_DIRECT("KEYSCAN Two Keys Press Detected: (%d, %d), (%d, %d)",
                           keyscan_current_data.key[0].row, keyscan_current_data.key[0].column,
                           keyscan_current_data.key[1].row, keyscan_current_data.key[1].column);
            }

            /* Handle the first key press event */
            if (is_first_pressed == false)
            {
                DBG_DIRECT("KEYSCAN First Key Press Detected");
                is_first_pressed = true;

                /*
                 * Switch Trigger Source:
                 * Change from "Key Trigger" to "Register Bit Trigger" to allow
                 * the TIMER to periodically trigger scans (polling).
                 */
                KEYSCAN_SetManualSelect(KEYSCAN, KEYSCAN_MANUAL_SEL_BIT);

                /* Enable RAP Mode for RTC and KEYSCAN */
                RTC_RAPModeCmd(ENABLE);
                KEYSCAN_RAPModeCmd(KEYSCAN, ENABLE);

                /* Start RTC to begin periodic scanning */
                RTC_ActionTrigger(RTC_ACTION_START);
            }
        }
        /* If KEYSCAN FIFO is empty, all keys have been released. */
        else
        {
            /* Stop RTC and Disable RAP Modes */
            RTC_ActionTrigger(RTC_ACTION_STOP);
            RTC_RAPModeCmd(DISABLE);
            KEYSCAN_RAPModeCmd(KEYSCAN, DISABLE);

            DBG_DIRECT("KEYSCAN: All Keys Released");

            /* Reset RTC for next event */
            RTC_ResetCounter();
            RTC_SetCompValue(RTC_COMP_NUM, RTC_COMP_VALUE);

            /* Re-initialize KEYSCAN to trigger on physical Key Press (Edge Detect) again. */
            driver_keyscan_init(KEYSCAN_MANUAL_SEL_KEY);
            KEYSCAN_Cmd(KEYSCAN, ENABLE);

            /* Reset global data */
            is_first_pressed = false;
            keyscan_current_data.length = 0;
            memset(keyscan_current_data.key, 0, sizeof(keyscan_current_data.key));

            return;
        }

        /* Clear Pending and Unmask Interrupt to allow next scan */
        KEYSCAN_ClearINTPendingBit(KEYSCAN, KEYSCAN_INT_SCAN_END);
        KEYSCAN_INTMask(KEYSCAN, KEYSCAN_INT_SCAN_END, DISABLE);
    }
}
