/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: LicenseRef-Realtek-5-Clause
 */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "log_core.h"
#include "rtl_gpio.h"
#include "rtl_nvic.h"
#include "rtl_pinmux.h"
#include "rtl_rap.h"
#include "rtl_rcc.h"
#include "rtl_timer.h"
#include "rtl_ir.h"
#include "rtl_dma.h"

/* Defines -------------------------------------------------------------------*/
/* IR configuration which can be modified based on requirements */
#define IR_RX_PIN                           P2_5

/*
 * IR RX Configuration:
 *
 * DMA WATERLEVEL:
 *    Determines the FIFO threshold for triggering DMA requests.
 *    When FIFO data >= WATERLEVEL, a DMA request is asserted.
 *
 * COUNT THRESHOLD (End of Packet Detection):
 *    Threshold: 0x1000 ticks = ~102.4us (@ 40MHz).
 *    Logic: This value must be greater than the PWM low period (50us) to ensure
 *           the timeout interrupt triggers only when the signal stops (End of Packet).
 */
#define IR_RX_DMA_WATERLEVEL                1
#define IR_RX_COUNT_THRESHOLD               0x2000

/* DMA configuration which can be modified based on requirements */
#define IR_RX_DMA_CHANNEL                   DMA_CH3
#define IR_RX_DMA_CHANNEL_NUM               DMA_CH_NUM3
#define IR_RX_DMA_CHANNEL_IRQN              DMA0_CH3_IRQn
#define IR_RX_DMA_TRANSFER_SIZE             40
/* RAP Configuration */
#define IR_RX_DMA_ACTION_TRANSFER           RAP_ACTION_DMA_CHANNEL_EN(3)

/* PWM configuration which can be modified based on requirements */
#define PWM_OUT_PIN                         P2_4
#define PWM_OUT_PINMUX                      PWM0
#define PWM_OUT_TIMER                       TIMER1_CH0
/* RAP Configuration */
#define PWM_OUT_ACTION_START                RAP_ACTION_TIMER_START(1, 0)
#define PWM_OUT_ACTION_STOP                 RAP_ACTION_TIMER_STOP(1, 0)

/*
 * Configure PWM period and duty cycle parameters.
 *
 * Target PWM: 10KHz Frequency, 50% Duty Cycle.
 *  - PWM_PERIOD     = 40,000,000 / 10,000 = 4000
 *  - PWM_HIGH_COUNT = 4000 * 50% = 2000
 */
#define PWM_OUT_PERIOD                      4000
#define PWM_OUT_HIGH_COUNT                  2000

/* TIMER configuration which can be modified based on requirements */

/*
 * Event for triggering PWM START Action to output carrier waveform.
 * Set the period as 40000000 which keep the carrier on for 1 second.
 */
#define TIMER_START_CARRIER_NUM             TIMER1_CH1
#define TIMER_START_CARRIER_PERIOD          40000000
/* RAP Configuration */
#define TIMER_START_CARRIER_EVENT_TIMEOUT   RAP_EVENT_TIMER_TIMEOUT(1, 1)

/*
 * Event for triggering PWM START Action to output carrier waveform.
 * Set the period as 400000 which keep the carrier on for 10ms.
 */
#define TIMER_STOP_CARRIER_NUM              TIMER1_CH2
#define TIMER_STOP_CARRIER_PERIOD           420000
/* RAP Configuration */
#define TIMER_STOP_CARRIER_EVENT_TIMEOUT    RAP_EVENT_TIMER_TIMEOUT(1, 2)
#define TIMER_STOP_CARRIER_ACTION_START     RAP_ACTION_TIMER_START(1, 2)
#define TIMER_STOP_CARRIER_ACTION_STOP      RAP_ACTION_TIMER_STOP(1, 2)

/* Globals -------------------------------------------------------------------*/
static uint32_t ir_rx_dma_recv_buffer[4096];
static volatile uint16_t ir_rx_dma_total_len = 0;
volatile bool is_receive_done = false;

/* Functions -----------------------------------------------------------------*/

/**
 * \brief  IR Interrupt Service Routine (ISR) prototype.
 */
static void IR_Handler(void);

/**
 * \brief  DMA Channel Interrupt Service Routine (ISR) prototype.
 */
static void IR_RX_DMA_Handler(void);

/**
 * \brief  TIMER START CARRIER Interrupt Service Routine (ISR) prototype.
 */
static void TIMER_START_CARRIER_Handler(void);

/**
 * \brief  TIMER STOP CARRIER Interrupt Service Routine (ISR) prototype.
 */
static void TIMER_STOP_CARRIER_Handler(void);

/**
 * \brief  Initializes pad and pinmux settings for IR.
 */
static void board_ir_init(void)
{
    Pad_Config(IR_RX_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pinmux_Config(IR_RX_PIN, IRDA_RX);
}

/**
 * \brief  Initialize IR peripheral with DMA support.
 */
static void driver_ir_init(void)
{
    /* Enable IR clock */
    RCC_ClockCmd(IR_CLOCK, ENABLE);

    /* Configure IR parameters */
    IR_InitTypeDef IR_InitStruct;
    IR_StructInit(&IR_InitStruct);
    IR_InitStruct.IR_Freq               = 40000000;
    IR_InitStruct.IR_Mode               = IR_MODE_RX;
    IR_InitStruct.IR_RxStartMode        = IR_RX_AUTO_MODE;
    IR_InitStruct.IR_RxFIFOFullCtrl     = IR_RX_FIFO_FULL_DISCARD_NEWEST;
    IR_InitStruct.IR_RxFilterTime       = IR_RX_FILTER_TIME_50NS;
    IR_InitStruct.IR_RxTriggerMode      = IR_RX_FALL_EDGE;
    IR_InitStruct.IR_RxCntThrType       = IR_RX_COUNT_HIGH_LEVEL;
    IR_InitStruct.IR_RxCntThr           = IR_RX_COUNT_THRESHOLD;
    IR_InitStruct.IR_RxDMAEn            = ENABLE;
    IR_InitStruct.IR_RxWaterLevel       = IR_RX_DMA_WATERLEVEL;
    IR_Init(&IR_InitStruct);

    /* Update vector table with ISR */
    ram_vector_table_update(IR_IRQn, IR_Handler);

    /* Configure NVIC */
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = IR_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_Init(&NVIC_InitStruct);

    /*
     * Enable IR Count Threshold Interrupt (End of Packet):
     * - IR_INT_RX_CNT_THR: Triggered when RX idle duration >= IR_RX_COUNT_THRESHOLD.
     *
     * Note: Data transfer is handled by DMA, so FIFO level interrupts are not required here.
     */
    IR_MaskINTConfig(IR_INT_RX_CNT_THR, DISABLE);
    IR_INTConfig(IR_INT_RX_CNT_THR, ENABLE);

    /* Clear RX FIFO */
    IR_ClearRxFIFO();
}

/**
 * \brief  Initialize DMA for IR RX.
 */
static void driver_ir_dma_init(void)
{
    /* Enable DMA clock */
    RCC_ClockCmd(DMA_CLOCK, ENABLE);

    /* Configure DMA parameters */
    DMA_InitTypeDef DMA_InitStruct;
    DMA_StructInit(&DMA_InitStruct);
    DMA_InitStruct.DMA_ChannelNum          = IR_RX_DMA_CHANNEL_NUM;
    DMA_InitStruct.DMA_BufferSize          = IR_RX_DMA_TRANSFER_SIZE;
    DMA_InitStruct.DMA_Direction           = DMA_DIR_PERIPHERAL_TO_MEMORY;
    DMA_InitStruct.DMA_SourceDataSize      = DMA_DATA_SIZE_WORD;
    DMA_InitStruct.DMA_DestinationDataSize = DMA_DATA_SIZE_WORD;
    DMA_InitStruct.DMA_SourceMsize         = DMA_MSIZE_1;
    DMA_InitStruct.DMA_DestinationMsize    = DMA_MSIZE_1;
    DMA_InitStruct.DMA_SourceInc           = DMA_SOURCE_FIX;
    DMA_InitStruct.DMA_DestinationInc      = DMA_DESTINATION_INC;
    DMA_InitStruct.DMA_SourceAddr          = (uint32_t)(&IR->IR_RX_FIFO);
    DMA_InitStruct.DMA_DestinationAddr     = (uint32_t)(ir_rx_dma_recv_buffer);
    DMA_InitStruct.DMA_SourceHandshake     = DMA_HANDSHAKE_IR_RX;
    DMA_Init(IR_RX_DMA_CHANNEL, &DMA_InitStruct);

    /* Update vector table with ISR */
    ram_vector_table_update(IR_RX_DMA_CHANNEL_IRQN, IR_RX_DMA_Handler);

    /* Configure NVIC */
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel         = IR_RX_DMA_CHANNEL_IRQN;
    NVIC_InitStruct.NVIC_IRQChannelCmd      = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_Init(&NVIC_InitStruct);

    /* Enable Transfer Done Interrupt */
    DMA_INTConfig(IR_RX_DMA_CHANNEL_NUM, DMA_INT_TRANSFER, ENABLE);

    /* Enable DMA Transfer */
    DMA_Cmd(IR_RX_DMA_CHANNEL_NUM, ENABLE);
}

/**
 * \brief  Initializes pad and pinmux settings for PWM.
 */
static void board_pwm_init(void)
{
    Pad_Config(PWM_OUT_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_HIGH);
    Pinmux_Config(PWM_OUT_PIN, PWM_OUT_PINMUX);
}

/**
 * \brief  Initialize TIMER for PWM Generation of 10K waveform.
 */
static void driver_pwm_init(void)
{
    /* Clock already enabled by duration timer init */
    RCC_ClockCmd(TIMER1_CLOCK, ENABLE);

    /* Configure PWM parameters */
    TIMER_TimeBaseInitTypeDef TIMER_InitStruct;
    TIMER_StructInit(&TIMER_InitStruct);
    TIMER_InitStruct.TIMER_Mode     = TIMER_MODE_USERDEFINE;
    TIMER_InitStruct.PWM_En         = ENABLE;
    TIMER_InitStruct.TIMER_Period   = PWM_OUT_PERIOD;
    TIMER_InitStruct.PWM_HighCount  = PWM_OUT_HIGH_COUNT;
    TIMER_InitStruct.PWM_Polarity   = PWM_POLARITY_HIGH;
    TIMER_TimeBaseInit(PWM_OUT_TIMER, &TIMER_InitStruct);
}

/**
 * \brief  Initialize TIMER START CARRIER (2s Period).
 */
static void driver_timer_start_carrier_init(void)
{
    /* Enable TIMER clock */
    RCC_ClockCmd(TIMER1_CLOCK, ENABLE);

    /* Configure TIMER parameters */
    TIMER_TimeBaseInitTypeDef TIMER_InitStruct;
    TIMER_StructInit(&TIMER_InitStruct);
    TIMER_InitStruct.TIMER_Mode     = TIMER_MODE_USERDEFINE;
    TIMER_InitStruct.PWM_En         = DISABLE;
    TIMER_InitStruct.TIMER_Period   = TIMER_START_CARRIER_PERIOD;
    TIMER_TimeBaseInit(TIMER_START_CARRIER_NUM, &TIMER_InitStruct);

    /*
     * When the trigger conditions are met, the system supports the simultaneous
     * triggering of RAP events and interrupts. Users can enable the interrupt
     * function according to their actual usage needs.
     * Set to 1 to activate the interrupt function.
     */
#if 0
    /* Update vector table with ISR */
    ram_vector_table_update(TIMER1_CH1_IRQn, TIMER_START_CARRIER_Handler);

    /* Configure NVIC */
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel         = TIMER1_CH1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd      = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_Init(&NVIC_InitStruct);

    /* Enable TIMER Interrupt */
    TIMER_INTConfig(TIMER_START_CARRIER_NUM, TIMER_INT_TIMEOUT, ENABLE);
#endif
}

/**
 * \brief  Initialize TIMER STOP CARRIER (1s Period).
 */
static void driver_timer_stop_carrier_init(void)
{
    /* Enable TIMER clock */
    RCC_ClockCmd(TIMER1_CLOCK, ENABLE);

    /* Configure TIMER parameters */
    TIMER_TimeBaseInitTypeDef TIMER_InitStruct;
    TIMER_StructInit(&TIMER_InitStruct);
    TIMER_InitStruct.TIMER_Mode     = TIMER_MODE_USERDEFINE;
    TIMER_InitStruct.PWM_En         = DISABLE;
    TIMER_InitStruct.TIMER_Period   = TIMER_STOP_CARRIER_PERIOD;
    TIMER_TimeBaseInit(TIMER_STOP_CARRIER_NUM, &TIMER_InitStruct);

    /*
     * When the trigger conditions are met, the system supports the simultaneous
     * triggering of RAP events and interrupts. Users can enable the interrupt
     * function according to their actual usage needs.
     * Set to 1 to activate the interrupt function.
     */
#if 0
    /* Update vector table with ISR */
    ram_vector_table_update(TIMER1_CH0_IRQn, TIMER_STOP_CARRIER_Handler);

    /* Configure NVIC */
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel         = TIMER1_CH0_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd      = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_Init(&NVIC_InitStruct);

    /* Enable TIMER Interrupt */
    TIMER_INTConfig(TIMER_STOP_CARRIER_NUM, TIMER_INT_TIMEOUT, ENABLE);
#endif
}

/**
 * \brief  Main entry.
 */
int main(void)
{
    /* Enable Global Interrupts */
    __enable_irq();

    DBG_DIRECT("Start RAP timer timeout tirrger IR RX start sample");

    /* Peripheral initialization */
    board_ir_init();
    driver_ir_init();
    driver_ir_dma_init();
    board_pwm_init();
    driver_pwm_init();
    driver_timer_start_carrier_init();
    driver_timer_stop_carrier_init();

    /* Channel Allocate */
    uint8_t channel0, channel1;
    RAP_ChannelAllocate(&channel0);
    RAP_ChannelAllocate(&channel1);

    /* Handles the "Start Carrier" event for Start Output Waveform  */
    RAP_EventRouteSet(TIMER_START_CARRIER_EVENT_TIMEOUT, channel0);
    RAP_ActionBindSet(TIMER_STOP_CARRIER_ACTION_START, channel0);
    RAP_ActionBindSet(PWM_OUT_ACTION_START, channel0);
    /* The Event trigger IR RX Action at the same time */
    RAP_ActionBindSet(RAP_ACTION_IR_START_RX, channel0);

    /* Handles the "Stop Carrier" event for Stop Output Waveform  */
    RAP_EventRouteSet(TIMER_STOP_CARRIER_EVENT_TIMEOUT, channel1);
    RAP_ActionBindSet(TIMER_STOP_CARRIER_ACTION_STOP, channel1);
    RAP_ActionBindSet(PWM_OUT_ACTION_STOP, channel1);

    /* Enable RAP Mode For Peripherals */
    TIMER_RAPModeCmd(TIMER_START_CARRIER_NUM, ENABLE);
    TIMER_RAPModeCmd(TIMER_STOP_CARRIER_NUM, ENABLE);
    TIMER_RAPModeCmd(PWM_OUT_TIMER, ENABLE);
    IR_RAPModeCmd(ENABLE);

    /* Trigger the first event manually to start the loop */
    TIMER_ActionTrigger(TIMER_START_CARRIER_NUM, TIMER_ACTION_START);

    while (1)
    {
        /*
         * Check data reception status.
         * The 'is_receive_done' flag is set in the IR ISR (Count Threshold Interrupt).
         * Count Threshold Interrupt is triggered indicates IR packet reception complete.
         */
        if (is_receive_done)
        {
            /* Clear the flag immediately to acknowledge processing */
            is_receive_done = false;

            DBG_DIRECT("IR RX Length: %d", ir_rx_dma_total_len);

            /* Print received IR data */
            for (uint16_t i = 0; i < ir_rx_dma_total_len; i++)
            {
                DBG_DIRECT("IR RX Data[%d]: 0x%08X", i, ir_rx_dma_recv_buffer[i]);
            }

            /* Reset buffer and length for next reception */
            memset(ir_rx_dma_recv_buffer, 0, sizeof(ir_rx_dma_recv_buffer));
            ir_rx_dma_total_len = 0;
        }
    }
}

/**
 * \brief  DMA Channel Interrupt Service Routine.
 *         Called when IR_RX_DMA_TRANSFER_SIZE words have been moved to RAM.
 */
static void IR_RX_DMA_Handler(void)
{
    /* Update total received length */
    ir_rx_dma_total_len += IR_RX_DMA_TRANSFER_SIZE;

    /* Clear interrupt pending bit */
    DMA_ClearINTPendingBit(IR_RX_DMA_CHANNEL_NUM, DMA_INT_TRANSFER);

    /* Update DMA Destination Address to the next free buffer block */
    DMA_SetDestinationAddress(IR_RX_DMA_CHANNEL, (uint32_t)&ir_rx_dma_recv_buffer[ir_rx_dma_total_len]);

    /* Re-enable DMA for the next transfer block */
    DMA_Cmd(IR_RX_DMA_CHANNEL_NUM, ENABLE);
}

/**
 * \brief  IR Interrupt Service Routine.
 *         Handles packet completion (Idle Timeout).
 */
static void IR_Handler(void)
{
    uint16_t ir_fifo_remain_len = 0;
    uint16_t dma_transfer_len = 0;

    /* Get Interrupt status */
    ITStatus int_status_rx_count_thrd = IR_GetINTStatus(IR_INT_RX_CNT_THR);

    /* Mask IR interrupts to prevent re-entry during processing */
    IR_MaskINTConfig(IR_INT_RX_CNT_THR, ENABLE);

    /* Check for Idle Timeout (End of Packet) */
    if (int_status_rx_count_thrd == SET)
    {
        DBG_DIRECT("IR_INT_RX_CNT_THR (IR RX Complete)");

        /* Suspend DMA */
        DMA_SafeSuspend(IR_RX_DMA_CHANNEL);

        /* Get the current number of DMA transferred */
        dma_transfer_len = DMA_GetTransferLen(IR_RX_DMA_CHANNEL) / 4;

        /* Read remaining data in the IR FIFO that didn't trigger a DMA request */
        ir_fifo_remain_len = IR_GetRxDataLen();
        IR_ReceiveBuf(&ir_rx_dma_recv_buffer[ir_rx_dma_total_len + dma_transfer_len], ir_fifo_remain_len);

        /* Update total received length */
        ir_rx_dma_total_len += (dma_transfer_len + ir_fifo_remain_len);

        /* Stop Reception */
        IR_Cmd(IR_MODE_RX, DISABLE);
        DMA_Cmd(IR_RX_DMA_CHANNEL_NUM, DISABLE);

        /* Set flag to notify main loop that packet is ready */
        is_receive_done = true;

        DMA_SetDestinationAddress(IR_RX_DMA_CHANNEL, (uint32_t)&ir_rx_dma_recv_buffer[ir_rx_dma_total_len]);
        DMA_Cmd(IR_RX_DMA_CHANNEL_NUM, ENABLE);

        /* Clear Interrupt Pending Bit */
        IR_ClearINTPendingBit(IR_INT_RX_CNT_THR_CLR);
    }

    /* Unmask IR interrupts */
    IR_MaskINTConfig(IR_INT_RX_CNT_THR, DISABLE);
}

/**
 * \brief  TIMER START CARRIER Interrupt Service Routine (ISR) prototype.
 */
static void TIMER_START_CARRIER_Handler(void)
{
    DBG_DIRECT("TIMER_START_CARRIER_Handler");

    if (TIMER_GetINTStatus(TIMER_START_CARRIER_NUM, TIMER_INT_TIMEOUT) == SET)
    {
        TIMER_ClearINT(TIMER_START_CARRIER_NUM, TIMER_INT_TIMEOUT);
    }
}

/**
 * \brief  TIMER STOP CARRIER Interrupt Service Routine (ISR) prototype.
 */
static void TIMER_STOP_CARRIER_Handler(void)
{
    DBG_DIRECT("TIMER_STOP_CARRIER_Handler");

    if (TIMER_GetINTStatus(TIMER_STOP_CARRIER_NUM, TIMER_INT_TIMEOUT) == SET)
    {
        TIMER_ClearINT(TIMER_STOP_CARRIER_NUM, TIMER_INT_TIMEOUT);
    }
}
