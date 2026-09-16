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
#include "rtl_dma.h"
#include "utils.h"
#include "log_core.h"

/* Defines -------------------------------------------------------------------*/

/* TIMER Latch Handling Configuration */
/*
 * Select the mechanism used to retrieve data from the TIMER Latch FIFO.
 *
 * - SAMPLE_CONFIG_TIMER_LATCH_MODE_ISR:
 *   When the Latch FIFO reaches the threshold, an interrupt is triggered.
 *   The CPU reads the FIFO in the ISR to clear the status.
 *
 * - SAMPLE_CONFIG_TIMER_LATCH_MODE_DMA:
 *   When the Latch FIFO reaches the threshold, a DMA request is triggered.
 *   The DMA controller automatically transfers data from the FIFO to memory.
 */
#define SAMPLE_CONFIG_TIMER_LATCH_MODE_ISR       1
#define SAMPLE_CONFIG_TIMER_LATCH_MODE_DMA       0

#if ((SAMPLE_CONFIG_TIMER_LATCH_MODE_ISR + \
      SAMPLE_CONFIG_TIMER_LATCH_MODE_DMA) != 1)
#error "SAMPLE CONFIG ERROR!!!"
#endif

/* TIMER configuration which can be modified based on requirements */
#define TIMER_NUM                       TIMER2_CH0
#define TIMER_IRQN                      TIMER2_CH0_IRQn

/*
 * Configure TIMER Latch Threshold parameters.
 *
 * The Latch event is triggered when the count of captured pulses in the FIFO
 * reaches this threshold.
 *
 * Note: If DMA is used:
 * - DMA Msize must equal the Latch Pulse Threshold.
 * - DMA Transfer Size should be a multiple of this threshold to ensure
 *   proper interrupt generation upon transfer completion.
 */
#define TIMER_LATCH_TRIGGER_THRESHOLD   4
#define TIMER_LATCH_TRIGGER_PAD         P0_1

/* RAP Configuration */
#define TIMER_EVENT_LATCH_THRESHOLD     RAP_EVENT_TIMER_LATCH_FIFO_THRD(2, 0)

/* DMA configuration which can be modified based on requirements */
#define DMA_CHANNEL                     DMA_CH1
#define DMA_CHANNEL_NUM                 DMA_CH_NUM1
#define DMA_CHANNEL_IRQN                DMA0_CH1_IRQn

/* GPIO configuration which can be modified based on requirements */
#define OUTPUT_PIN                      P0_0
#define GPIO_OUT_PIN                    GPIO_GetPinBit(OUTPUT_PIN)
#define GPIO_OUT_PORT                   GPIO_GetPort(OUTPUT_PIN)
/* RAP Configuration */
#define GPIO_OUT_ACTION_TOGGLE          RAP_ACTION_GPIOA_DRTOGGLE(0)

/* Globals -------------------------------------------------------------------*/
uint32_t latch_fifo_buffer[32];

#if (SAMPLE_CONFIG_TIMER_LATCH_MODE_ISR == 1)
/* TIMER ISR Mode: Store captured data */
uint32_t timer_latch_data[8] = {0};
uint8_t timer_latch_len = 0;
volatile bool is_receive_done = false;
#endif

#if (SAMPLE_CONFIG_TIMER_LATCH_MODE_DMA == 1)
/* DMA Mode: Store transfer length */
uint16_t dma_transfer_len = 0;
volatile bool is_receive_done = false;
#endif

/* Functions -----------------------------------------------------------------*/
/**
 * \brief  TIMER Interrupt Service Routine (ISR) prototype.
 */
void TIMER_Handler(void);

/**
 * \brief  DMA Interrupt Service Routine (ISR) prototype.
 */
void DMA_Handler(void);

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
    RCC_ClockCmd(TIMER2_CLOCK, ENABLE);

    /* Configure TIMER parameters */
    TIMER_TimeBaseInitTypeDef TIMER_InitStruct;
    TIMER_StructInit(&TIMER_InitStruct);
    TIMER_InitStruct.TIMER_ClockSrc = TIMER_CLOCK_SRC_40M;
    TIMER_InitStruct.TIMER_ClockDiv = TIMER_CLOCK_DIV_1;
    TIMER_InitStruct.TIMER_Mode     = TIMER_MODE_FREERUN;

    /* Configure TIMER Latch parameters */
    TIMER_InitStruct.TIMER_Latch.TIMER_LatchEn[0] = ENABLE;
    TIMER_InitStruct.TIMER_Latch.TIMER_LatchTriggerMode[0] = TIMER_LATCH_TRIGGER_FALLING_EDGE;
    TIMER_InitStruct.TIMER_Latch.TIMER_LatchThreshold = TIMER_LATCH_TRIGGER_THRESHOLD;
    TIMER_InitStruct.TIMER_Latch.TIMER_LatchTriggerPad = TIMER_LATCH_TRIGGER_PAD;

#if (SAMPLE_CONFIG_TIMER_LATCH_MODE_DMA == 1)
    /* Enable TIMER DMA Request */
    TIMER_InitStruct.TIMER_DMAEn = ENABLE;
    TIMER_InitStruct.TIMER_DMATarget = TIMER_DMA_LATCH_FIFO;
#endif

    TIMER_TimeBaseInit(TIMER_NUM, &TIMER_InitStruct);

    /* Clear TIMER Latch FIFO */
    TIMER_ClearFIFO(TIMER_NUM, TIMER_CLEAR_LATCH_FIFO);

#if (SAMPLE_CONFIG_TIMER_LATCH_MODE_ISR == 1)
    /* Update vector table with ISR */
    ram_vector_table_update(TIMER_IRQN, TIMER_Handler);

    /* Configure NVIC */
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel         = TIMER_IRQN;
    NVIC_InitStruct.NVIC_IRQChannelCmd      = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_Init(&NVIC_InitStruct);

    /* Enable TIMER Latch Threshold Interrupt */
    TIMER_INTConfig(TIMER_NUM, TIMER_INT_LATCH_FIFO_THRESHOLD, ENABLE);
#endif
}

/**
 * \brief  Initializes DMA peripheral.
 */
static void driver_dma_init(void)
{
#if (SAMPLE_CONFIG_TIMER_LATCH_MODE_DMA == 1)
    /* Enable DMA clock */
    RCC_ClockCmd(DMA_CLOCK, ENABLE);

    /* Configure DMA parameters */
    DMA_InitTypeDef DMA_InitStruct;
    DMA_StructInit(&DMA_InitStruct);
    DMA_InitStruct.DMA_ChannelNum          = DMA_CHANNEL_NUM;
    DMA_InitStruct.DMA_SourceDataSize      = DMA_DATA_SIZE_WORD;
    DMA_InitStruct.DMA_DestinationDataSize = DMA_DATA_SIZE_WORD;
    DMA_InitStruct.DMA_SourceMsize         = DMA_MSIZE_4;
    DMA_InitStruct.DMA_DestinationMsize    = DMA_MSIZE_4;
    DMA_InitStruct.DMA_SourceHandshake     = DMA_HANDSHAKE_TIMER2_TRX;
    DMA_InitStruct.DMA_Direction           = DMA_DIR_PERIPHERAL_TO_MEMORY;
    DMA_InitStruct.DMA_SourceInc           = DMA_SOURCE_FIX;
    DMA_InitStruct.DMA_DestinationInc      = DMA_DESTINATION_INC;
    DMA_InitStruct.DMA_SourceAddr          = (uint32_t) & (TIMER_NUM->TIMER_LAT_CNT_0);
    DMA_InitStruct.DMA_BufferSize          = TIMER_LATCH_TRIGGER_THRESHOLD;
    DMA_InitStruct.DMA_DestinationAddr     = (uint32_t)latch_fifo_buffer;
    DMA_Init(DMA_CHANNEL, &DMA_InitStruct);

    /* Update vector table and NVIC */
    ram_vector_table_update(DMA_CHANNEL_IRQN, DMA_Handler);

    /* Configure NVIC */
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel         = DMA_CHANNEL_IRQN;
    NVIC_InitStruct.NVIC_IRQChannelCmd      = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_Init(&NVIC_InitStruct);

    /* Configure DMA Interrupt */
    DMA_INTConfig(DMA_CHANNEL_NUM, DMA_INT_TRANSFER, ENABLE);

    /* Enable DMA channel */
    DMA_Cmd(DMA_CHANNEL_NUM, ENABLE);
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

    DBG_DIRECT("Start RAP timer latch trigger gpio toggle sample");

    /* Peripheral initialization */
    board_gpio_init();
    driver_gpio_init();
    driver_dma_init();
    driver_timer_init();

    /* Configure RAP channel */
    uint8_t channel0;
    RAP_ChannelAllocate(&channel0);

    /* Route TIMER Latch Threshold Event to RAP channel */
    RAP_EventRouteSet(TIMER_EVENT_LATCH_THRESHOLD, channel0);
    /* Bind GPIO Toggle Action to RAP channel */
    RAP_ActionBindSet(GPIO_OUT_ACTION_TOGGLE, channel0);

    /* Enable RAP Mode for TIMER and GPIO */
    TIMER_RAPModeCmd(TIMER_NUM, ENABLE);
    GPIO_RAPModeCmd(GPIO_OUT_PORT, GPIO_OUT_PIN, ENABLE);

    /* Start TIMER */
    TIMER_ActionTrigger(TIMER_NUM, TIMER_ACTION_START);

    /*
     * Simulate pulse input using P0_2 (User can short P0_2 to TIMER_LATCH_TRIGGER_PAD).
     * When TIMER_LATCH_TRIGGER_PAD detects 4 rising-edge pulses (TIMER_LATCH_TRIGGER_THRESHOLD),
     * a RAP event will be triggered to toggle the GPIO.
     */
    pad_generate_pulse(P0_2, TIMER_LATCH_TRIGGER_THRESHOLD);

    while (1)
    {
#if (SAMPLE_CONFIG_TIMER_LATCH_MODE_ISR == 1)
        /*
         * Check TIMER Latch data reception status.
         * The 'is_receive_done' flag is set in the TIMER ISR.
         */
        if (is_receive_done)
        {
            /* Clear the flag immediately to acknowledge processing */
            is_receive_done = false;

            /* Print captured latch data */
            for (uint8_t i = 0; i < timer_latch_len; i++)
            {
                DBG_DIRECT("  timer_latch_data[%d] = 0x%08x", i, timer_latch_data[i]);
            }

            /* Clear data buffer */
            memset(timer_latch_data, 0, sizeof(timer_latch_data));
            timer_latch_len = 0;
        }
#endif

#if (SAMPLE_CONFIG_TIMER_LATCH_MODE_DMA == 1)
        /*
         * Check DMA transfer completion status.
         * The 'is_receive_done' flag is set in the DMA ISR.
         */
        if (is_receive_done)
        {
            /* Clear the flag immediately to acknowledge processing */
            is_receive_done = false;

            for (uint16_t i = 0; i < dma_transfer_len / 4; i++)
            {
                DBG_DIRECT("dma_data[%d] = 0x%x", i, latch_fifo_buffer[i]);
            }

            /* Clear data buffer */
            memset(latch_fifo_buffer, 0, sizeof(latch_fifo_buffer));
            dma_transfer_len = 0;
        }
#endif

    }

    return 0;
}

#if (SAMPLE_CONFIG_TIMER_LATCH_MODE_ISR == 1)
/**
 * \brief  TIMER Interrupt Service Routine (ISR).
 *         Executed if SAMPLE_CONFIG_TIMER_LATCH_MODE_ISR is enabled.
 */
void TIMER_Handler(void)
{
    DBG_DIRECT("TIMER_Handler");

    if (TIMER_GetINTStatus(TIMER_NUM, TIMER_INT_LATCH_FIFO_THRESHOLD))
    {
        timer_latch_len = TIMER_GetLatchFIFOLength(TIMER_NUM);

        /* Read data from FIFO to clear the threshold condition */
        TIMER_GetLatchFIFO(TIMER_NUM, timer_latch_data, timer_latch_len);

        /* Clear Interrupt Status */
        TIMER_ClearINT(TIMER_NUM, TIMER_INT_LATCH_FIFO_THRESHOLD);

        /* Set flag to notify main loop */
        is_receive_done = true;
    }
}
#endif

#if (SAMPLE_CONFIG_TIMER_LATCH_MODE_DMA == 1)
/**
 * \brief  DMA Interrupt Service Routine (ISR).
 *         Executed if SAMPLE_CONFIG_TIMER_LATCH_MODE_DMA is enabled.
 */
void DMA_Handler(void)
{
    DBG_DIRECT("DMA_Handler");

    /* Check transfer length */
    dma_transfer_len = DMA_GetTransferLen(DMA_CHANNEL);

    /* Set flag to notify main loop */
    is_receive_done = true;

    /* Clear DMA Interrupts */
    DMA_ClearAllTypeINT(DMA_CHANNEL_NUM);
}
#endif