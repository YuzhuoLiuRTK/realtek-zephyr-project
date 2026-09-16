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
#include "mem_config.h"
#include "rtl_dma.h"
#include "rtl_spi.h"
#include "rtl_rtc.h"
#include "utils.h"
#include "log_core.h"
#include "lis3dh_driver.h"

/* Defines ------------------------------------------------------------*/
/* GPIO configuration which can be modified based on requirements */
#define INPUT_PIN                           P0_0
#define GPIO_IN_PIN                         GPIO_GetPinBit(INPUT_PIN)
#define GPIO_IN_PORT                        GPIO_GetPort(INPUT_PIN)
#define GPIO_IN_IRQN                        GPIOA0_IRQn
/* RAP Configuration */
#define GPIO_IN_EVENT_IN                    RAP_EVENT_GPIOA(0)

/* SPI configuration */
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

/* RAP configuration */
#define SPI_MASTER_EVENT_TRANSFER_START     RAP_EVENT_SPI_START(0)
#define SPI_MASTER_ACTION_TRANSFER_START    RAP_ACTION_SPI_START(0)

/* DMA configuration */
/* Master RX DMA */
#define SPI_MASTER_RX_DMA_CHANNEL           DMA_CH4
#define SPI_MASTER_RX_DMA_CHANNEL_NUM       DMA_CH_NUM4
#define SPI_MASTER_RX_DMA_IRQN              DMA0_CH4_IRQn
#define SPI_MASTER_RX_DMA_HANDSHAKE         DMA_HANDSHAKE_SPI0_RX

/* RAP configuration */
#define SPI_MASTER_RX_DMA_ACTION_START      RAP_ACTION_DMA_CHANNEL_EN(4)

/*
 * 3 Axis includes X, Y, Z.
 * 12-bit data out per Axis in high resolution mode (12-bits).
 */
#define BYTES_PER_SAMPLE                    (3 * 2)

/* Number of sets to read in one trigger. */
#define TRIGGER_SAMPLE_NUM                  (10)

/* DMA Block Counter (Interrupt triggers after this many blocks) */
#define DMA_BLOCK_COUNTER_NUM               (10)

/* Globals ------------------------------------------------------------------*/
/*
 * SPI RX Buffer Size Calculation:
 * In Full Duplex Mode, for every transaction set:
 * 1 Byte (Command Phase) + N Bytes (Data Phase).
 */
static uint8_t spi_rx_buffer[(1 + BYTES_PER_SAMPLE * TRIGGER_SAMPLE_NUM) * DMA_BLOCK_COUNTER_NUM] = {0};
volatile bool is_receive_done = false;

/* Functions ----------------------------------------------------------------*/
/**
 * \brief  DMA Interrupt Service Routine (ISR) prototype for SPI Master RX.
 */
static void SPI_MASTER_RX_DMA_Handler(void);

/**
 * \brief  GPIO Interrupt Service Routine (ISR) prototype.
 */
static void GPIO_Pin_Handler(void);

/**
 * \brief  Initializes pad and pinmux settings.
 */
static void board_gpio_init(void)
{
    Pad_Config(INPUT_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_DISABLE, PAD_OUT_HIGH);
    Pinmux_Config(INPUT_PIN, DWGPIO);
}

/**
 * \brief  Initializes GPIO peripheral and Interrupts.
 */
static void driver_gpio_init(void)
{
    /* Enable GPIO clock */
    RCC_ClockCmd(GPIOA_CLOCK, ENABLE);

    /* Configure GPIO parameters as input mode */
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin        = GPIO_IN_PIN;
    GPIO_InitStruct.GPIO_Dir        = GPIO_DIR_IN;
    GPIO_InitStruct.GPIO_INTEventEn = ENABLE;
    GPIO_InitStruct.GPIO_Trigger    = GPIO_TRIGGER_EDGE;
    GPIO_InitStruct.GPIO_Polarity   = GPIO_POLARITY_ACTIVE_HIGH;
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

    GPIO_ClearINTPendingBit(GPIO_IN_PORT, GPIO_IN_PIN);
    GPIO_MaskINTConfig(GPIO_IN_PORT, GPIO_IN_PIN, DISABLE);
#endif
}

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
    SPI_InitStruct.SPI_RxWaterlevel      = 1;
    SPI_InitStruct.SPI_RxThresholdLevel  = 2;
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
 * \brief  Main entry.
 */
int main(void)
{
    /* Enable Global Interrupts */
    __enable_irq();

    DBG_DIRECT("Start Sensor gpio trigger spi transfer sample");

    /* Peripheral initialization */
    board_gpio_init();
    driver_gpio_init();
    board_spi_master_init();
    driver_spi_master_init();
    driver_dma_master_rx_init();

    /* Initialize Sensor */
    lis3dh_init(SPI_MASTER);

    /* Read ID */
    uint8_t id = lis3dh_read_id(SPI_MASTER);
    DBG_DIRECT("Sensor ID: 0x%02x", id);

    /* Configure Sensor FIFO & Interrupt */
    lis3dh_config_fifo(SPI_MASTER, ENABLE, TRIGGER_SAMPLE_NUM * BYTES_PER_SAMPLE);
    lis3dh_config_interrupt(SPI_MASTER);

    /* Configure RAP channel */
    uint8_t channel0, channel1;
    RAP_ChannelAllocate(&channel0);
    RAP_ChannelAllocate(&channel1);

    /* GPIO in event trigger SPI transfer start */
    RAP_EventRouteSet(GPIO_IN_EVENT_IN, channel0);
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
    GPIO_RAPModeCmd(GPIO_IN_PORT, GPIO_IN_PIN, ENABLE);
    SPI_RAPModeCmd(SPI_MASTER, ENABLE);
    DMA_RAPModeCmd(SPI_MASTER_RX_DMA_CHANNEL, ENABLE);

    while (1)
    {
        /*
         * Check data reception status.
         * The 'is_receive_done' flag is set in the RX DMA ISR.
         * RX DMA Interrupt is triggered indicates a complete packet.
         */
        if (is_receive_done)
        {
            /* Clear the flag immediately to acknowledge processing */
            is_receive_done = false;

            for (uint8_t j = 0; j < DMA_BLOCK_COUNTER_NUM; j++)
            {
                uint32_t block_offset = j * (BYTES_PER_SAMPLE * TRIGGER_SAMPLE_NUM + 1);
                for (uint8_t i = 0; i < TRIGGER_SAMPLE_NUM; i++)
                {
                    uint32_t data_offset = i * BYTES_PER_SAMPLE + 1;
                    DBG_DIRECT("data%d: x:%d, y:%d, z:%d", j * TRIGGER_SAMPLE_NUM + i,
                               spi_rx_buffer[0 + data_offset + block_offset] | (spi_rx_buffer[1 + data_offset + block_offset] <<
                                                                                8),
                               spi_rx_buffer[2 + data_offset + block_offset] | (spi_rx_buffer[3 + data_offset + block_offset] <<
                                                                                8),
                               spi_rx_buffer[4 + data_offset + block_offset] | (spi_rx_buffer[5 + data_offset + block_offset] <<
                                                                                8));
                }
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

        /* Disable triggers to stop reading */
        GPIO_RAPModeCmd(GPIO_IN_PORT, GPIO_IN_PIN, DISABLE);
        DMA_RAPModeCmd(SPI_MASTER_RX_DMA_CHANNEL, DISABLE);
    }
}

/**
 * \brief  GPIO Interrupt Service Routine (ISR).
 */
static void GPIO_Pin_Handler()
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
