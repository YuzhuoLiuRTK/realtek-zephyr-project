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
#include "rtl_i2c.h"
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

/* I2C configuration which can be modified based on requirements */
/* Configure I2C Master parameters */
#define I2C_MASTER                          I2C0
#define I2C_MASTER_CLOCK                    I2C0_CLOCK
#define I2C_MASTER_SCL_PIN                  P4_0
#define I2C_MASTER_SDA_PIN                  P4_1
#define I2C_MASTER_SCL_PINMUX               I2C0_CLK
#define I2C_MASTER_SDA_PINMUX               I2C0_DAT

#define LIS3DH_I2C_SA0_PIN                  P4_2
#define LIS3DH_I2C_CS_PIN                   P4_3

/* RAP configuration */
#define I2C_MASTER_EVENT_TRANSFER_START     RAP_EVENT_I2C_STARTED(0)
#define I2C_MASTER_ACTION_TRANSFER_START    RAP_ACTION_I2C_START(0)

/* DMA configuration which can be modified based on requirements */
/* Master RX DMA (Receives data from Slave) */
#define I2C_MASTER_RX_DMA_CHANNEL           DMA_CH4
#define I2C_MASTER_RX_DMA_CHANNEL_NUM       DMA_CH_NUM4
#define I2C_MASTER_RX_DMA_IRQN              DMA0_CH4_IRQn
#define I2C_MASTER_RX_DMA_HANDSHAKE         DMA_HANDSHAKE_I2C0_RX
/* RAP configuration */
#define I2C_MASTER_RX_DMA_ACTION_START      RAP_ACTION_DMA_CHANNEL_EN(4)

/*
 * 3 Axis includes X, Y, Z.
 * 12-bit data out per Axis in high resolution mode (12-bits).
 */
#define BYTES_PER_SAMPLE                    (3 * 2)

/*
 * Number of sets to read in one trigger.
 * Original logic: 3 sets * 6 bytes = 18 bytes per trigger.
 */
#define TRIGGER_SAMPLE_NUM                  (3)

/* DMA Block Counter (Interrupt triggers after this many blocks) */
#define DMA_BLOCK_COUNTER_NUM               (10)

/* Globals ------------------------------------------------------------------*/
static uint8_t i2c_rx_buffer[BYTES_PER_SAMPLE * TRIGGER_SAMPLE_NUM * DMA_BLOCK_COUNTER_NUM] = {0};
volatile bool is_receive_done = false;

/* Functions ----------------------------------------------------------------*/
/**
 * \brief  DMA Interrupt Service Routine (ISR) prototype for Master RX.
 */
static void I2C_MASTER_RX_DMA_Handler(void);

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
 * \brief  Initialize I2C Master pad and pinmux settings.
 */
static void board_i2c_master_init(void)
{
    Pad_Config(I2C_MASTER_SCL_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE,
               PAD_OUT_HIGH);
    Pad_Config(I2C_MASTER_SDA_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE,
               PAD_OUT_HIGH);

    Pinmux_Config(I2C_MASTER_SCL_PIN, I2C_MASTER_SCL_PINMUX);
    Pinmux_Config(I2C_MASTER_SDA_PIN, I2C_MASTER_SDA_PINMUX);

    Pad_Config(LIS3DH_I2C_SA0_PIN, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_ENABLE,
               PAD_OUT_LOW);
    Pad_Config(LIS3DH_I2C_CS_PIN, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE,
               PAD_OUT_HIGH);
}

/**
 * \brief  Initialize I2C Master peripheral.
 */
static void driver_i2c_master_init(void)
{
    /* Enable I2C Master clock */
    RCC_ClockCmd(I2C_MASTER_CLOCK, ENABLE);

    /* Configure I2C Master parameters */
    I2C_InitTypeDef I2C_InitStruct;
    I2C_StructInit(&I2C_InitStruct);
    I2C_InitStruct.I2C_ClockSpeed   = 400000;
    I2C_InitStruct.I2C_DeviceMode   = I2C_DEVICE_MODE_MASTER;
    I2C_InitStruct.I2C_AddressMode  = I2C_ADDRESS_MODE_7BIT;
    I2C_InitStruct.I2C_SlaveAddress = LIS3DH_I2C_ADDRESS;
    I2C_InitStruct.I2C_Ack          = ENABLE;
    I2C_InitStruct.I2C_RxDMAEn      = DISABLE;
    I2C_InitStruct.I2C_RxWaterlevel = 1;
    I2C_Init(I2C_MASTER, &I2C_InitStruct);

    /* Enable I2C */
    I2C_Cmd(I2C_MASTER, ENABLE);

    I2C_RAPModeCmd(I2C_MASTER, ENABLE);
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
    DMA_InitStruct.DMA_ChannelNum          = I2C_MASTER_RX_DMA_CHANNEL_NUM;
    DMA_InitStruct.DMA_Direction           = DMA_DIR_PERIPHERAL_TO_MEMORY;
    DMA_InitStruct.DMA_BufferSize          = 0; /* To be configured later */
    DMA_InitStruct.DMA_SourceInc           = DMA_SOURCE_FIX;
    DMA_InitStruct.DMA_DestinationInc      = DMA_DESTINATION_INC;
    DMA_InitStruct.DMA_SourceDataSize      = DMA_DATA_SIZE_BYTE;
    DMA_InitStruct.DMA_DestinationDataSize = DMA_DATA_SIZE_BYTE;
    DMA_InitStruct.DMA_SourceMsize         = DMA_MSIZE_1;
    DMA_InitStruct.DMA_DestinationMsize    = DMA_MSIZE_1;
    DMA_InitStruct.DMA_SourceAddr          = (uint32_t)(&(I2C_MASTER->IC_DATA_CMD));
    DMA_InitStruct.DMA_DestinationAddr     = (uint32_t)NULL;
    DMA_InitStruct.DMA_SourceHandshake     = I2C_MASTER_RX_DMA_HANDSHAKE;
    DMA_Init(I2C_MASTER_RX_DMA_CHANNEL, &DMA_InitStruct);

    /* Update vector table with ISR */
    ram_vector_table_update(I2C_MASTER_RX_DMA_IRQN, I2C_MASTER_RX_DMA_Handler);

    /* Configure NVIC */
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel         = I2C_MASTER_RX_DMA_IRQN;
    NVIC_InitStruct.NVIC_IRQChannelCmd      = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_Init(&NVIC_InitStruct);

    /* Enable DMA Block Counter interrupt */
    DMA_INTConfig(I2C_MASTER_RX_DMA_CHANNEL_NUM, DMA_INT_BLOCK_COUNTER, ENABLE);
}

/**
 * \brief  Main entry.
 */
int main(void)
{
    /* Enable Global Interrupts */
    __enable_irq();

    DBG_DIRECT("Start Sensor gpio trigger i2c transfer sample");

    /* Peripheral initialization */
    board_gpio_init();
    driver_gpio_init();
    board_i2c_master_init();
    driver_i2c_master_init();
    driver_dma_master_rx_init();

    /* Initialize Sensor */
    lis3dh_init(I2C_MASTER);

    /* Read ID */
    uint8_t id = lis3dh_read_id(I2C_MASTER);
    DBG_DIRECT("Sensor ID: 0x%02x", id);

    /* Configure Sensor FIFO (3 samples * 6 bytes = 18) & Interrupt */
    lis3dh_config_fifo(I2C_MASTER, ENABLE, TRIGGER_SAMPLE_NUM * BYTES_PER_SAMPLE);
    lis3dh_config_interrupt(I2C_MASTER);

    /* Configure RAP channel */
    uint8_t channel0, channel1;
    RAP_ChannelAllocate(&channel0);
    RAP_ChannelAllocate(&channel1);

    /* GPIO in event trigger I2C transfer start */
    RAP_EventRouteSet(GPIO_IN_EVENT_IN, channel0);
    RAP_ActionBindSet(I2C_MASTER_ACTION_TRANSFER_START, channel0);

    /* I2C transfer start event trigger DMA start */
    RAP_EventRouteSet(I2C_MASTER_EVENT_TRANSFER_START, channel1);
    RAP_ActionBindSet(I2C_MASTER_RX_DMA_ACTION_START, channel1);

    /*
     * Configure Burst Read Protocol
     * Tell I2C to read starting from OUT_X_L with Auto-Increment
     */
    uint8_t cmd = LIS3DH_REG_OUT_X_L | LIS3DH_I2C_MS_BIT;
    lis3dh_config_burst_read(I2C_MASTER, cmd, TRIGGER_SAMPLE_NUM * BYTES_PER_SAMPLE);

    /*
     * Configure DMA Transport
     * Buffer Size = 18 bytes, Block Counter = 10
     */
    DMA_SetBufferSize(I2C_MASTER_RX_DMA_CHANNEL, TRIGGER_SAMPLE_NUM * BYTES_PER_SAMPLE);
    DMA_SetDestinationAddress(I2C_MASTER_RX_DMA_CHANNEL, (uint32_t)(i2c_rx_buffer));
    DMA_SetBlockCounter(I2C_MASTER_RX_DMA_CHANNEL, DMA_BLOCK_COUNTER_NUM);
    DMA_ContDarCmd(I2C_MASTER_RX_DMA_CHANNEL, ENABLE);

    /* Enable RAP Mode */
    GPIO_RAPModeCmd(GPIO_IN_PORT, GPIO_IN_PIN, ENABLE);
    DMA_RAPModeCmd(I2C_MASTER_RX_DMA_CHANNEL, ENABLE);

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

            /*
             * Note: The printing logic below strictly follows the original sample code.
             * It iterates through the buffer based on block index 'j'.
             */
            for (uint8_t j = 0; j < DMA_BLOCK_COUNTER_NUM; j++)
            {
                uint32_t block_offset = j * BYTES_PER_SAMPLE * TRIGGER_SAMPLE_NUM;
                for (uint8_t i = 0; i < TRIGGER_SAMPLE_NUM; i++)
                {
                    uint32_t data_offset = i * BYTES_PER_SAMPLE;
                    DBG_DIRECT("data%d: x:%d, y:%d, z:%d", j * TRIGGER_SAMPLE_NUM + i,
                               i2c_rx_buffer[0 + data_offset + block_offset] | (i2c_rx_buffer[1 + data_offset + block_offset] <<
                                                                                8),
                               i2c_rx_buffer[2 + data_offset + block_offset] | (i2c_rx_buffer[3 + data_offset + block_offset] <<
                                                                                8),
                               i2c_rx_buffer[4 + data_offset + block_offset] | (i2c_rx_buffer[5 + data_offset + block_offset] <<
                                                                                8));
                }
            }
        }
    }

    return 0;
}

/**
 * \brief  Master RX DMA Interrupt Service Routine (ISR).
 */
static void I2C_MASTER_RX_DMA_Handler(void)
{
    DBG_DIRECT("DMA_GetTransferLen %d", DMA_GetTransferLen(I2C_MASTER_RX_DMA_CHANNEL));

    if (DMA_GetINTStatus(I2C_MASTER_RX_DMA_CHANNEL_NUM, DMA_INT_BLOCK_COUNTER) == ENABLE)
    {
        DBG_DIRECT("DMA Block Counter Interrupt");

        /* Notify main loop that this packet is received done */
        is_receive_done = true;

        DMA_ClearAllTypeINT(I2C_MASTER_RX_DMA_CHANNEL_NUM);

        /* Disable triggers to stop reading */
        GPIO_RAPModeCmd(GPIO_IN_PORT, GPIO_IN_PIN, DISABLE);
        DMA_RAPModeCmd(I2C_MASTER_RX_DMA_CHANNEL, DISABLE);
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
