/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: LicenseRef-Realtek-5-Clause
 */

/*============================================================================*
 *                              Header Files
 *============================================================================*/
#include <stdlib.h>
#include <stdint.h>
#include "rtl_pinmux.h"
#include "rtl_rcc.h"
#include "rtl_nvic.h"
#include "rtl_rap.h"
#include "rtl_timer.h"
#include "rtl_gpio.h"
#include "rtl_dma.h"
#include "rtl_spi.h"
#include "rtl_rtc.h"
#include "utils.h"
#include "log_core.h"
#include "lis3dh_driver.h"

/* Defines ------------------------------------------------------------*/
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
 * Define the RTC_PSC_VALUE as (320 - 1) which generates a 10kHz(10ms) tick.
 */
#define RTC_PRESCALER_VALUE                 (320 - 1)

/*
 * Configure RTC Compare and Reload parameters.
 *
 * Calculation formula:
 *  - Timeout = Value * (1 / Tick_Freq).
 *
 * Based on the following settings:
 *  - RTC Tick Frequency: 100Hz (from Prescaler settings).
 *
 * Define RTC_COMP_VALUE as 20 which timeout is 200ms.
 * Define RTC_COMP_RELOAD_VALUE as 10 which reload time is 100ms.
 */
#define RTC_COMP_NUM                        RTC_COMP0
#define RTC_COMP_INT                        RTC_INT_COMP0
#define RTC_COMP_VALUE                      10
#define RTC_COMP_RELOAD_VALUE               10

/* RAP Configuration */
#define RTC_EVENT_COMPARE                   RAP_EVENT_RTC_COMPARE(0)

/* SPI configuration which can be modified based on requirements */
#define SPI_MASTER                          SPI0
#define SPI_MASTER_CLOCK                    SPI0_CLOCK
#define SPI_MASTER_SCK_PIN                  P4_0
#define SPI_MASTER_MOSI_PIN                 P4_1
#define SPI_MASTER_MISO_PIN                 P4_2
#define SPI_MASTER_CS_PIN                   P4_3
#define SPI_MASTER_CLK_PINMUX               SPI0_CLK_MASTER
#define SPI_MASTER_MO_PINMUX                SPI0_MO_MASTER
#define SPI_MASTER_MI_PINMUX                SPI0_MI_MASTER
#define SPI_MASTER_CS_PINMUX                SPI0_SS_N_0_MASTER

/* RAP Configuration */
#define SPI_MASTER_EVENT_TRANSFER_START     RAP_EVENT_SPI_START(0)
#define SPI_MASTER_ACTION_TRANSFER_START    RAP_ACTION_SPI_START(0)

/* DMA configuration which can be modified based on requirements */
/* Master RX DMA */
#define SPI_MASTER_RX_DMA_CHANNEL           DMA_CH4
#define SPI_MASTER_RX_DMA_CHANNEL_NUM       DMA_CH_NUM4
#define SPI_MASTER_RX_DMA_IRQN              DMA0_CH4_IRQn
#define SPI_MASTER_RX_DMA_HANDSHAKE         DMA_HANDSHAKE_SPI0_RX

/* RAP Configuration */
#define SPI_MASTER_RX_DMA_ACTION_START      RAP_ACTION_DMA_CHANNEL_EN(4)

/*
 * 3 Axis includes X, Y, Z.
 * 12-bit data out per Axis in high resolution mode (2 bytes).
 */
#define BYTES_PER_SAMPLE                    (3 * 2)

/*
 * Number of sets to read in one trigger.
 * In Periodic mode, we usually read 1 sample set per RTC trigger.
 */
#define TRIGGER_SAMPLE_NUM                  (1)

/* DMA Block Counter (Interrupt triggers after this many blocks) */
#define DMA_BLOCK_COUNTER_NUM               (10)

/* Globals ------------------------------------------------------------------*/
/*
 * SPI RX Buffer Size Calculation:
 * In Full Duplex Mode, for every transaction set:
 * 1 Byte (Command Phase) + N Bytes (Data Phase).
 * Buffer stores 'DMA_BLOCK_COUNTER_NUM' history of samples.
 */
static uint8_t spi_rx_buffer[(1 + BYTES_PER_SAMPLE * TRIGGER_SAMPLE_NUM) * DMA_BLOCK_COUNTER_NUM] = {0};
static volatile bool is_receive_done = false;

/* Functions ----------------------------------------------------------------*/
/**
 * \brief  DMA Interrupt Service Routine (ISR) prototype for SPI Master RX.
 */
static void SPI_MASTER_RX_DMA_Handler(void);

/**
 * \brief  RTC Interrupt Service Routine (ISR) prototype.
 */
static void RTC_Handler(void);

/**
 * \brief  Initialize SPI Master pad and pinmux settings.
 */
static void board_spi_master_init(void)
{
    Pad_Config(SPI_MASTER_SCK_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE,
               PAD_OUT_HIGH);
    Pad_Config(SPI_MASTER_MOSI_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE,
               PAD_OUT_HIGH);
    Pad_Config(SPI_MASTER_MISO_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE,
               PAD_OUT_HIGH);
    Pad_Config(SPI_MASTER_CS_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE,
               PAD_OUT_HIGH);

    Pinmux_Config(SPI_MASTER_SCK_PIN, SPI_MASTER_CLK_PINMUX);
    Pinmux_Config(SPI_MASTER_MOSI_PIN, SPI_MASTER_MO_PINMUX);
    Pinmux_Config(SPI_MASTER_MISO_PIN, SPI_MASTER_MI_PINMUX);
    Pinmux_Config(SPI_MASTER_CS_PIN, SPI_MASTER_CS_PINMUX);
}

/**
 * \brief  Initialize SPI Master peripheral.
 */
static void driver_spi_master_init(void)
{
    /* Enable SPI Master clock */
    SPI_DeInit(SPI_MASTER);
    RCC_ClockCmd(SPI_MASTER_CLOCK, ENABLE);

    /* Configure SPI Master parameters */
    SPI_InitTypeDef  SPI_InitStruct;
    SPI_StructInit(&SPI_InitStruct);
    SPI_InitStruct.SPI_Direction         = SPI_DIRECTION_FULLDUPLEX;
    SPI_InitStruct.SPI_Mode              = SPI_MODE_MASTER;
    SPI_InitStruct.SPI_DataSize          = SPI_DATA_SIZE_8b;
    SPI_InitStruct.SPI_CPOL              = SPI_CPOL_LOW;
    SPI_InitStruct.SPI_CPHA              = SPI_CPHA_1EDGE;
    SPI_InitStruct.SPI_FrameFormat       = SPI_FRAME_MOTOROLA;
    SPI_InitStruct.SPI_BaudRatePrescaler = 100;
    SPI_InitStruct.SPI_WrapModeEn        = ENABLE;
    SPI_InitStruct.SPI_TXNDF             = 8;
    SPI_InitStruct.SPI_RxDMAEn           = DISABLE; /* Enable when needed */
    SPI_InitStruct.SPI_RxWaterlevel      = 1 - 1;
    SPI_InitStruct.SPI_RxThresholdLevel  = 2;
    /* Manual Mode: WrapModeDMAEn is DISABLE (Default) */
    SPI_Init(SPI_MASTER, &SPI_InitStruct);

    /* Enable SPI */
    SPI_Cmd(SPI_MASTER, ENABLE);
}

/**
 * \brief  Initialize DMA Channel for Master RX.
 */
static void driver_dma_master_rx_init(void)
{
    /* Enable DMA clock */
    RCC_ClockCmd(DMA_CLOCK, ENABLE);

    /* Configure DMA parameters */
    DMA_InitTypeDef DMA_InitStruct;
    DMA_StructInit(&DMA_InitStruct);
    DMA_InitStruct.DMA_ChannelNum          = SPI_MASTER_RX_DMA_CHANNEL_NUM;
    DMA_InitStruct.DMA_Direction           = DMA_DIR_PERIPHERAL_TO_MEMORY;
    DMA_InitStruct.DMA_BufferSize          = 0; /* To be configured later */
    DMA_InitStruct.DMA_SourceInc           = DMA_SOURCE_FIX;
    DMA_InitStruct.DMA_DestinationInc      = DMA_DESTINATION_INC;
    DMA_InitStruct.DMA_SourceDataSize      = DMA_DATA_SIZE_BYTE;
    DMA_InitStruct.DMA_DestinationDataSize = DMA_DATA_SIZE_BYTE;
    DMA_InitStruct.DMA_SourceMsize         = DMA_MSIZE_1;
    DMA_InitStruct.DMA_DestinationMsize    = DMA_MSIZE_1;
    DMA_InitStruct.DMA_SourceAddr          = (uint32_t)SPI_MASTER->SPI_DR;
    DMA_InitStruct.DMA_DestinationAddr     = (uint32_t)NULL;
    DMA_InitStruct.DMA_SourceHandshake     = SPI_MASTER_RX_DMA_HANDSHAKE;
    DMA_Init(SPI_MASTER_RX_DMA_CHANNEL, &DMA_InitStruct);

    /* Update vector table with ISR */
    ram_vector_table_update(SPI_MASTER_RX_DMA_IRQN, SPI_MASTER_RX_DMA_Handler);

    /* Configure NVIC */
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel         = SPI_MASTER_RX_DMA_IRQN;
    NVIC_InitStruct.NVIC_IRQChannelCmd      = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 2;
    NVIC_Init(&NVIC_InitStruct);

    /* Enable DMA Block Counter interrupt */
    DMA_INTConfig(SPI_MASTER_RX_DMA_CHANNEL_NUM, DMA_INT_BLOCK_COUNTER, ENABLE);
}

/**
 * \brief  Initialize RTC for Periodic Trigger.
 */
static void driver_rtc_init(void)
{
    /* Enable RTC clock */
    RCC_ClockCmd(RTC_CLOCK, ENABLE);
    RTC_DeInit();

    /* Configure RTC prescaler to generate the tick */
    RTC_SetPrescaler(RTC_PRESCALER_VALUE);

    /* Configure RTC compare and reload value */
    RTC_EnableCompAutoReload(RTC_COMP_NUM, RTC_COMP_VALUE, RTC_COMP_RELOAD_VALUE);

    /* Reset the RTC counter to ensure the initial timing starts from zero */
    RTC_ResetCounter();

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

    DBG_DIRECT("Start Sensor RTC trigger spi transfer sample");

    /* Peripheral initialization */
    board_spi_master_init();
    driver_spi_master_init();
    driver_dma_master_rx_init();
    driver_rtc_init();

    /* Initialize Sensor */
    lis3dh_init(SPI_MASTER);

    /* Read ID */
    uint8_t id = lis3dh_read_id(SPI_MASTER);
    DBG_DIRECT("Sensor ID: 0x%02x", id);

    /* Configure Sensor FIFO */
    lis3dh_config_fifo(SPI_MASTER, DISABLE, 0);

    /* Configure RAP channel */
    uint8_t channel0, channel1;
    RAP_ChannelAllocate(&channel0);
    RAP_ChannelAllocate(&channel1);

    /* RTC Compare event trigger SPI transfer start */
    RAP_EventRouteSet(RTC_EVENT_COMPARE, channel0);
    RAP_ActionBindSet(SPI_MASTER_ACTION_TRANSFER_START, channel0);

    /* SPI transfer start event trigger RX DMA start */
    RAP_EventRouteSet(SPI_MASTER_EVENT_TRANSFER_START, channel1);
    RAP_ActionBindSet(SPI_MASTER_RX_DMA_ACTION_START, channel1);

    /*
     * Configure Burst Read Protocol
     * Prepare Command Byte (Read | MS | Addr)
     * Calculate total transfer length: 1 cmd byte + Data Bytes
     * Manual Mode: Driver will fill TX FIFO once.
     */
    uint8_t cmd = LIS3DH_REG_OUT_X_L | LIS3DH_SPI_RW_BIT | LIS3DH_SPI_MS_BIT;
    lis3dh_config_burst_read(SPI_MASTER, cmd, TRIGGER_SAMPLE_NUM * BYTES_PER_SAMPLE);

    /* Configure RX DMA to receive (Dummy + Data) */
    DMA_SetBufferSize(SPI_MASTER_RX_DMA_CHANNEL, TRIGGER_SAMPLE_NUM * BYTES_PER_SAMPLE + 1);
    DMA_SetDestinationAddress(SPI_MASTER_RX_DMA_CHANNEL, (uint32_t)spi_rx_buffer);
    DMA_SetBlockCounter(SPI_MASTER_RX_DMA_CHANNEL, DMA_BLOCK_COUNTER_NUM);
    DMA_ContDarCmd(SPI_MASTER_RX_DMA_CHANNEL, ENABLE);

    /* Enable RAP Mode */
    RTC_RAPModeCmd(ENABLE);
    SPI_RAPModeCmd(SPI_MASTER, ENABLE);
    DMA_RAPModeCmd(SPI_MASTER_RX_DMA_CHANNEL, ENABLE);

    /* Start RTC Trigger */
    RTC_ActionTrigger(RTC_ACTION_START);

    while (1)
    {
        /*
         * Check data reception status.
         * The 'is_dma_receive_done' flag is set in the DMA ISR (Block Counter Interrupt).
         * Block Counter Interrupt is triggered indicates a batch of data received.
         */
        if (is_receive_done)
        {
            /* Clear the flag immediately to acknowledge processing */
            is_receive_done = false;

            for (uint8_t j = 0; j < DMA_BLOCK_COUNTER_NUM; j++)
            {
                /*
                 * Indexing Correction for SPI Periodic Mode:
                 * Each trigger reads 1 set of data (6 bytes).
                 * Block Size = 1 (Dummy) + 6 (Data).
                 * Data Start = BlockOffset.
                 */
                uint32_t data_offset = j * (BYTES_PER_SAMPLE * TRIGGER_SAMPLE_NUM + 1) + 1;

                DBG_DIRECT("data%d: x:%d, y:%d, z:%d", j,
                           spi_rx_buffer[data_offset + 0] | (spi_rx_buffer[data_offset + 1] << 8),
                           spi_rx_buffer[data_offset + 2] | (spi_rx_buffer[data_offset + 3] << 8),
                           spi_rx_buffer[data_offset + 4] | (spi_rx_buffer[data_offset + 5] << 8));
            }
        }
    }

    return 0;
}

static void SPI_MASTER_RX_DMA_Handler(void)
{
    DBG_DIRECT("dma rx handler");

    if (DMA_GetINTStatus(SPI_MASTER_RX_DMA_CHANNEL_NUM, DMA_INT_BLOCK_COUNTER) == ENABLE)
    {
        DBG_DIRECT("DMA Block Counter Interrupt");

        /* Notify main loop that this packet is received done */
        is_receive_done = true;

        DMA_ClearAllTypeINT(SPI_MASTER_RX_DMA_CHANNEL_NUM);

        /* Stop Triggers */
        RTC_ActionTrigger(RTC_ACTION_STOP);
        RTC_RAPModeCmd(DISABLE);
        DMA_RAPModeCmd(SPI_MASTER_RX_DMA_CHANNEL, DISABLE);
    }
}

/**
 * \brief  RTC Interrupt Service Routine (ISR).
 */
static void RTC_Handler(void)
{
    if (RTC_GetINTStatus(RTC_COMP_INT) == SET)
    {
        DBG_DIRECT("RTC_HANDLER: RTC_COMP_NUM_INT");
        RTC_ClearCompINT(RTC_COMP_NUM);
    }
}
