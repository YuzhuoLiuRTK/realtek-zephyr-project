/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: LicenseRef-Realtek-5-Clause
 */

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <stdint.h>
#include "log_core.h"
#include "rtl_dma.h"
#include "rtl_gpio.h"
#include "rtl_ir.h"
#include "rtl_nvic.h"
#include "rtl_pinmux.h"
#include "rtl_rap.h"
#include "rtl_rcc.h"
#include "rtl_timer.h"

/* Defines -------------------------------------------------------------------*/
/* IR configuration which can be modified based on requirements */
#define IR_TX_PIN                           P2_5

/* DMA configuration which can be modified based on requirements */
#define IR_TX_DMA_CHANNEL                   DMA_CH3
#define IR_TX_DMA_CHANNEL_NUM               DMA_CH_NUM3
#define IR_TX_DMA_IRQN                      DMA0_CH3_IRQn
#define IR_TX_DMA_ACTION_TRANSFER           RAP_ACTION_DMA_CHANNEL_EN(3)
/*
 * Event for triggering IR TX START Action to output carrier waveform.
 * Set the period as 40000000 which outputs carrier every 1 second.
 */
#define TIMER_NUM                           TIMER1_CH0
#define TIMER_IRQN                          TIMER1_CH0_IRQn
#define TIMER_PERIOD                        40000000
/* RAP Configuration */
#define TIMER_EVENT_TIMEOUT                 RAP_EVENT_TIMER_TIMEOUT(1, 0)

/* Globals -------------------------------------------------------------------*/
static uint32_t ir_tx_data_buffer[100];

/* Functions -----------------------------------------------------------------*/

/**
 * \brief  TIMER Interrupt Service Routine (ISR) prototype.
 */
static void TIMER_Handler(void);

/**
 * \brief  DMA Interrupt Service Routine (ISR) prototype.
 */
static void IR_TX_DMA_Handler(void);

/**
 * \brief  Initializes pad and pinmux settings for IR.
 */
static void board_ir_init(void)
{
    Pad_Config(IR_TX_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_LOW);

    Pinmux_Config(IR_TX_PIN, IRDA_TX);
}

/**
 * \brief  Initialize IR peripheral.
 */
static void driver_ir_init(void)
{
    /* Enable IR clock */
    RCC_ClockCmd(IR_CLOCK, ENABLE);

    /* Configure IR parameters */
    IR_InitTypeDef IR_InitStruct;
    IR_StructInit(&IR_InitStruct);
    IR_InitStruct.IR_Freq           = 38000;
    IR_InitStruct.IR_DutyCycle      = 3;
    IR_InitStruct.IR_Mode           = IR_MODE_TX;
    IR_InitStruct.IR_TxInverse      = IR_TX_DATA_NORMAL;
    IR_InitStruct.IR_TxDMAEn        = ENABLE;
    IR_InitStruct.IR_TxWaterLevel   = IR_TX_FIFO_SIZE - 1;
    IR_Init(&IR_InitStruct);
}

/**
 * \brief  Initialize DMA peripheral for IR TX.
 */
static void driver_ir_dma_init(uint32_t transfer_len)
{
    /* Enable DMA clock */
    RCC_ClockCmd(DMA_CLOCK, ENABLE);

    /* Configure DMA parameters */
    DMA_InitTypeDef DMA_InitStruct;
    DMA_StructInit(&DMA_InitStruct);
    DMA_InitStruct.DMA_ChannelNum          = IR_TX_DMA_CHANNEL_NUM;
    DMA_InitStruct.DMA_BufferSize          = transfer_len;
    DMA_InitStruct.DMA_Direction           = DMA_DIR_MEMORY_TO_PERIPHERAL;
    DMA_InitStruct.DMA_SourceInc           = DMA_SOURCE_INC;
    DMA_InitStruct.DMA_DestinationInc      = DMA_DESTINATION_FIX;
    DMA_InitStruct.DMA_SourceDataSize      = DMA_DATA_SIZE_WORD;
    DMA_InitStruct.DMA_DestinationDataSize = DMA_DATA_SIZE_WORD;
    DMA_InitStruct.DMA_SourceMsize         = DMA_MSIZE_1;
    DMA_InitStruct.DMA_DestinationMsize    = DMA_MSIZE_1;
    DMA_InitStruct.DMA_SourceAddr          = (uint32_t)(ir_tx_data_buffer);
    DMA_InitStruct.DMA_DestinationAddr     = (uint32_t)(&IR->IR_TX_FIFO);
    DMA_InitStruct.DMA_DestHandshake       = DMA_HANDSHAKE_IR_TX;
    DMA_Init(IR_TX_DMA_CHANNEL, &DMA_InitStruct);

    /* Update vector table with ISR */
    ram_vector_table_update(IR_TX_DMA_IRQN, IR_TX_DMA_Handler);

    /* Configure NVIC */
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = IR_TX_DMA_IRQN;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_Init(&NVIC_InitStruct);

    /* Enable transfer interrupt */
    DMA_INTConfig(IR_TX_DMA_CHANNEL_NUM, DMA_INT_TRANSFER, ENABLE);
}

/**
 * \brief  Generate NEC Protocol Waveform into buffer.
 * \param  nec_data: 32-bit data to encode (LSB first usually).
 * \param  p_buf: Pointer to the destination buffer.
 * \return Total length of the generated waveform data.
 */
static uint16_t ir_generate_nec_waveform(uint32_t nec_data, uint32_t *p_buf)
{
    uint16_t index = 0;

    /*
     * Leader Code:
     * Format: 9ms Carrier (Mark) + 4.5ms Idle (Space)
     */
    p_buf[index++] = 0x80000000 | 342;  /* 9ms */
    p_buf[index++] = 0x00000000 | 171;  /* 4.5ms */

    /*
     * Data Codes:
     * Logic 0: 560us Mark + 560us Space
     * Logic 1: 560us Mark + 1.69ms Space
     */
    for (uint8_t i = 0; i < 32; i++)
    {
        /* Common Mark for both Logic 0 and Logic 1 */
        p_buf[index++] = 0x80000000 | 21; /* 560us */

        if (nec_data & (1UL << i))
        {
            /* Logic 1 Space */
            p_buf[index++] = 0x00000000 | 64; /* 1.69ms */
        }
        else
        {
            /* Logic 0 Space */
            p_buf[index++] = 0x00000000 | 21; /* 560us */
        }
    }

    /*
     * Stop Bit
     * Format: 560us Carrier (Mark) to terminate the transaction
     * Note: Set Bit 30 and Bit 31 to indicate this is the last data with carrier
     */
    p_buf[index++] = 0xC0000000 | 21; /* 560us */

    return index;
}

/**
 * \brief  Initializes TIMER peripheral.
 */
static void driver_timer_init(void)
{
    /* Enable TIMER Clocks */
    RCC_ClockCmd(TIMER1_CLOCK, ENABLE);

    /* Configure TIMER parameters */
    TIMER_TimeBaseInitTypeDef TIMER_InitStruct;
    TIMER_StructInit(&TIMER_InitStruct);
    TIMER_InitStruct.TIMER_Mode     = TIMER_MODE_USERDEFINE;
    TIMER_InitStruct.PWM_En         = DISABLE;
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
    ram_vector_table_update(TIMER_IRQN, TIMER_Trigger_Handler);

    /* Configure NVIC */
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = TIMER_IRQN;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_Init(&NVIC_InitStruct);

    /* Enable TIMER Interrupt */
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

    DBG_DIRECT("Start RAP timer timeout trigger ir tx sample");

    /* Initialize ir_tx_data_buffer (Waveform Generation) */
    uint32_t nec_payload = 0x00F720DF;
    uint32_t ir_tx_data_length = ir_generate_nec_waveform(nec_payload, ir_tx_data_buffer);

    /* Peripheral initialization */
    board_ir_init();
    driver_ir_init();
    driver_ir_dma_init(ir_tx_data_length);
    driver_timer_init();

    /* Channel Allocate */
    uint8_t channel0;
    RAP_ChannelAllocate(&channel0);

    /* Route TIMER Timeout Event to RAP channel */
    RAP_EventRouteSet(TIMER_EVENT_TIMEOUT, channel0);
    /* Bind IR START TX Action and TX DMA START Action to RAP channel */
    RAP_ActionBindSet(IR_TX_DMA_ACTION_TRANSFER, channel0);
    RAP_ActionBindSet(RAP_ACTION_IR_START_TX, channel0);

    /* Enable RAP Mode For Peripherals */
    TIMER_RAPModeCmd(TIMER_NUM, ENABLE);
    IR_RAPModeCmd(ENABLE);
    DMA_RAPModeCmd(IR_TX_DMA_CHANNEL, ENABLE);

    /* Start the Trigger TIMER to begin the loop */
    TIMER_ActionTrigger(TIMER_NUM, TIMER_ACTION_START);

    while (1)
    {
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
        TIMER_ClearINT(TIMER_NUM, TIMER_INT_TIMEOUT);
    }
}

/**
 * \brief  DMA Interrupt Service Routine (ISR).
 */
static void IR_TX_DMA_Handler(void)
{
    DBG_DIRECT("IR_TX_DMA_Handler: IR TX Done");

    /* Clear interrupt pending status */
    DMA_ClearINTPendingBit(IR_TX_DMA_CHANNEL_NUM, DMA_INT_TRANSFER);
}
