/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: LicenseRef-Realtek-5-Clause
 */

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <string.h>
#include "rtl_rcc.h"
#include "rtl_pinmux.h"
#include "rtl_nvic.h"
#include "rtl_gpio.h"
#include "rtl_timer.h"
#include "rtl_rap.h"
#include "rtl_spi.h"
#include "log_core.h"

/* Defines -------------------------------------------------------------------*/
/* SPI configuration which can be modified based on requirements */
/* Configure SPI Master parameters */
#define SPI_MASTER                  SPI1
#define SPI_MASTER_IRQN             SPI_1_IRQn
#define SPI_MASTER_CLOCK            SPI1_CLOCK
#define SPI_MASTER_SCK_PIN          P4_0
#define SPI_MASTER_MOSI_PIN         P4_1
#define SPI_MASTER_MISO_PIN         P4_2
#define SPI_MASTER_CS_PIN           P4_3
#define SPI_MASTER_SCK_PINMUX       SPI1_CLK_MASTER
#define SPI_MASTER_MOSI_PINMUX      SPI1_MO_MASTER
#define SPI_MASTER_MISO_PINMUX      SPI1_MI_MASTER
#define SPI_MASTER_CS_PINMUX        SPI1_SS_N_0_MASTER

/* Configure SPI Parameters */
#define SPI_WRAP_NDF_LEN            8   /* Number of Data Frames for Wrap Mode */
#define SPI_CMD_LENGTH              1   /* Command Length */
#define SPI_WAIT_COUNT              10  /* Time to wait for data reception */
#define SPI_RECV_LEN                SPI_WRAP_NDF_LEN - SPI_CMD_LENGTH  /* Receive length */

/* RAP Configuration */
#define SPI_MASTER_ACTION_START     RAP_ACTION_SPI_START(1)
#define SPI_MASTER_EVENT_START      RAP_EVENT_SPI_START(1)
#define SPI_MASTER_EVENT_END        RAP_EVENT_SPI_END(1)

/* TIMER configuration which can be modified based on requirements */
#define TIMER_NUM                   TIMER1_CH0
#define TIMER_IRQN                  TIMER1_CH0_IRQn
#define TIMER_CLOCK                 TIMER1_CLOCK

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
#define TIMER_PERIOD                (40000000)

/* RAP Configuration */
#define TIMER_EVENT_TIMEOUT         RAP_EVENT_TIMER_TIMEOUT(1, 0)

/* GPIO configuration which can be modified based on requirements */
#define OUTPUT_PIN                  P0_0
#define GPIO_OUT_PIN                GPIO_GetPinBit(OUTPUT_PIN)
#define GPIO_OUT_PORT               GPIO_GetPort(OUTPUT_PIN)
/* RAP Configuration */
#define GPIO_OUT_ACTION_TOGGLE      RAP_ACTION_GPIOA_DRTOGGLE(0)

/* Globals -------------------------------------------------------------------*/
static uint8_t spi_master_tx_buffer[50] = {0};
static uint8_t spi_master_rx_buffer[50] = {0};
static volatile uint8_t fifo_len = 0;
static volatile bool is_receive_done = false;

/* Functions -----------------------------------------------------------------*/

/**
 * \brief  TIMER Interrupt Service Routine (ISR) prototype.
 */
static void TIMER_Handler(void);

/**
 * \brief  SPI Master Interrupt Service Routine (ISR) prototype.
 */
static void SPI_MASTER_Handler(void);

/**
 * \brief  Initialize GPIO pad and pinmux settings.
 */
static void board_gpio_init(void)
{
    Pad_Config(OUTPUT_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE, PAD_OUT_HIGH);

    Pinmux_Config(OUTPUT_PIN, DWGPIO);
}

/**
 * \brief  Initialize GPIO peripheral.
 */
static void driver_gpio_init(void)
{
    /* Enable GPIO clock */
    RCC_ClockCmd(GPIOA_CLOCK, ENABLE);

    /* Configure GPIO parameters */
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin        = GPIO_OUT_PIN;
    GPIO_InitStruct.GPIO_Dir        = GPIO_DIR_OUT;
    GPIO_InitStruct.GPIO_INTEventEn = DISABLE;
    GPIO_Init(GPIO_OUT_PORT, &GPIO_InitStruct);
}

/**
 * \brief  Initialize TIMER peripheral.
 */
static void driver_timer_init(void)
{
    /* Enable TIMER clock */
    RCC_ClockCmd(TIMER_CLOCK, ENABLE);

    /* Configure TIMER parameters */
    TIMER_TimeBaseInitTypeDef TIMER_InitStruct;
    TIMER_StructInit(&TIMER_InitStruct);
    TIMER_InitStruct.TIMER_ClockSrc = TIMER_CLOCK_SRC_40M;
    TIMER_InitStruct.TIMER_ClockDiv = TIMER_CLOCK_DIV_1;
    TIMER_InitStruct.PWM_En         = DISABLE;
    TIMER_InitStruct.TIMER_Mode     = TIMER_MODE_USERDEFINE;
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
 * \brief  Initialize SPI pad and pinmux settings.
 */
static void board_spi_init(void)
{
    Pad_Config(SPI_MASTER_SCK_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE,
               PAD_OUT_HIGH);
    Pad_Config(SPI_MASTER_MOSI_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE,
               PAD_OUT_HIGH);
    Pad_Config(SPI_MASTER_MISO_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE,
               PAD_OUT_HIGH);
    Pad_Config(SPI_MASTER_CS_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE,
               PAD_OUT_HIGH);

    /* De-init pins to ensure clean state */
    Pinmux_Deinit(SPI_MASTER_SCK_PIN);
    Pinmux_Deinit(SPI_MASTER_MOSI_PIN);
    Pinmux_Deinit(SPI_MASTER_MISO_PIN);
    Pinmux_Deinit(SPI_MASTER_CS_PIN);

    /* De-init potential conflict pins */
    Pinmux_Deinit(P3_4);
    Pinmux_Deinit(P3_5);
    Pinmux_Deinit(P3_6);

    Pinmux_Config(SPI_MASTER_SCK_PIN, SPI_MASTER_SCK_PINMUX);
    Pinmux_Config(SPI_MASTER_MOSI_PIN, SPI_MASTER_MOSI_PINMUX);
    Pinmux_Config(SPI_MASTER_MISO_PIN, SPI_MASTER_MISO_PINMUX);
    Pinmux_Config(SPI_MASTER_CS_PIN, SPI_MASTER_CS_PINMUX);
}

/**
 * \brief  Initialize SPI peripheral.
 */
static void driver_spi_init(void)
{
    /* Enable SPI Master clock */
    RCC_ClockCmd(SPI_MASTER_CLOCK, ENABLE);

    /* Configure SPI Master parameters */
    SPI_InitTypeDef SPI_InitStruct;
    SPI_StructInit(&SPI_InitStruct);
    SPI_InitStruct.SPI_Direction         = SPI_DIRECTION_FULLDUPLEX;
    SPI_InitStruct.SPI_Mode              = SPI_MODE_MASTER;
    SPI_InitStruct.SPI_DataSize          = SPI_DATA_SIZE_8b;
    SPI_InitStruct.SPI_CPOL              = SPI_CPOL_LOW;
    SPI_InitStruct.SPI_CPHA              = SPI_CPHA_1EDGE;
    SPI_InitStruct.SPI_FrameFormat       = SPI_FRAME_MOTOROLA;
    SPI_InitStruct.SPI_BaudRatePrescaler = 40;
    SPI_InitStruct.SPI_WrapModeEn        = ENABLE;
    SPI_InitStruct.SPI_TXNDF             = SPI_WRAP_NDF_LEN;
    SPI_Init(SPI_MASTER, &SPI_InitStruct);

    /*
     * When the trigger conditions are met, the system supports the simultaneous
     * triggering of RAP events and interrupts. Users can enable the interrupt
     * function according to their actual usage needs.
     * Set to 1 to activate the interrupt function.
     */
#if 1
    /* Update vector table with ISR */
    ram_vector_table_update(SPI_MASTER_IRQN, SPI_MASTER_Handler);

    /* Configure NVIC */
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel         = SPI_MASTER_IRQN;
    NVIC_InitStruct.NVIC_IRQChannelCmd      = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_Init(&NVIC_InitStruct);

    /* Enable Wrap Mode TX Done interrupt */
    SPI_INTConfig(SPI_MASTER, SPI_INT_WRAP_TXD, ENABLE);
#endif

    /* Enable SPI Master */
    SPI_Cmd(SPI_MASTER, ENABLE);

    /* Initialize buffers */
    for (uint8_t i = 0; i < sizeof(spi_master_rx_buffer); i++)
    {
        spi_master_rx_buffer[i] = 0xFF;
    }
    spi_master_tx_buffer[0] = 0x9F;

    /* Configure SPI RAP Action Transfer parameters */
    /* Params: SPIx, CmdLength, WatiCount, TransferLength */
    SPI_SetActionTransfer(SPI_MASTER, SPI_CMD_LENGTH, SPI_WAIT_COUNT, SPI_RECV_LEN);

    /* Pre-fill TX FIFO with SPI_CMD_LENGTH byte */
    SPI_SendBuffer(SPI_MASTER, spi_master_tx_buffer, SPI_CMD_LENGTH);

    /* Set Wrap Mode NDF (Number of Data Frames) */
    SPI_WrapModeSetTxNdf(SPI_MASTER, SPI_WRAP_NDF_LEN);
}

/**
 * \brief  Main entry.
 */
int main(void)
{
    /* Enable Global Interrupts */
    __enable_irq();

    DBG_DIRECT("Start RAP timer timeout trigger SPI transfer sample");

    /* Peripheral board initialization */
    board_spi_init();
    board_gpio_init();

    /* Peripheral driver initialization */
    driver_spi_init();
    driver_gpio_init();
    driver_timer_init();

    /* Configure RAP channel */
    uint8_t channel0, channel1;
    RAP_ChannelAllocate(&channel0);
    RAP_ChannelAllocate(&channel1);

    /* Channel 0: Timer Timeout triggers SPI Start */
    RAP_EventRouteSet(TIMER_EVENT_TIMEOUT, channel0);
    RAP_ActionBindSet(SPI_MASTER_ACTION_START, channel0);

    /* Channel 1: SPI Start OR SPI End triggers GPIO Toggle */
    RAP_EventRouteSet(SPI_MASTER_EVENT_START, channel1);
    RAP_EventRouteSet(SPI_MASTER_EVENT_END, channel1);
    RAP_ActionBindSet(GPIO_OUT_ACTION_TOGGLE, channel1);

    /* Enable RAP Mode for peripherals */
    TIMER_RAPModeCmd(TIMER_NUM, ENABLE);
    SPI_RAPModeCmd(SPI_MASTER, ENABLE);
    GPIO_RAPModeCmd(GPIO_OUT_PORT, GPIO_OUT_PIN, ENABLE);

    /* Trigger the Timer to start the chain */
    TIMER_ActionTrigger(TIMER_NUM, TIMER_ACTION_START);

    while (1)
    {
        /*
         * Check data reception status.
         * The 'is_receive_done' flag is set in the SPI ISR (Wrap Mode TX Done Interrupt).
         * Wrap Mode TX Done Interrupt is triggered indicates SPI transfer complete.
         */
        if (is_receive_done)
        {
            /* Clear the flag immediately to acknowledge processing */
            is_receive_done = false;

            /* Print received SPI data */
            for (uint8_t i = 0; i < fifo_len; i++)
            {
                DBG_DIRECT("SPI Master RX Length: %d, Data[%d]: 0x%X", fifo_len, i, spi_master_rx_buffer[i]);
            }

            /* Reset buffer and length for next reception */
            memset(spi_master_rx_buffer, 0xFF, sizeof(spi_master_rx_buffer));
            fifo_len = 0;
        }
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
        TIMER_Cmd(TIMER_NUM, DISABLE);
        TIMER_ClearINT(TIMER_NUM, TIMER_INT_TIMEOUT);

        /* Add Test Code here */
        TIMER_Cmd(TIMER_NUM, ENABLE);
    }
}

/**
 * \brief  SPI Master Interrupt Service Routine (ISR).
 */
static void SPI_MASTER_Handler(void)
{
    DBG_DIRECT("SPI_MASTER_Handler");

    if (SPI_GetINTStatus(SPI_MASTER, SPI_INT_WRAP_TXD) == SET)
    {
        /* Clear interrupt pending bit */
        SPI_ClearINTPendingBit(SPI_MASTER, SPI_INT_WRAP_TXD);

        fifo_len = SPI_GetRxFIFOLen(SPI_MASTER);
        for (uint32_t i = 0; i < fifo_len; i++)
        {
            spi_master_rx_buffer[i] = SPI_ReceiveData(SPI_MASTER);
        }

        /* Set flag to notify main loop that data is ready */
        is_receive_done = true;
    }
}
