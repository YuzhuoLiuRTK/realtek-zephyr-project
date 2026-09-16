==================
Polling
==================
This sample application note describes using :term:`SPI3W` polling mode to read sensor ID.

Communicates with the sensor via SPI3W and reads the specified information by reading the data under the specified address.

Requirements
=============
For requirements, please refer to the :ref:`Requirements <general_requirements_en>`.
   
Wiring
==============

Connect the CLK, DATA, and CS pins on the EVB to the PWM3610DM-SUDU mouse module, and also connect GND and VDD. 
For specific pin configurations, refer to :ref:`Configurations<spi3w_polling_configuration_en>`.

Hardware Introduction
-----------------------
The PWM3610DM-SUDU is a laser sensor module. The PMW3610DM-SUDU registers are accessible through the serial port. The registers are used to read motion data and status, as well as to set the device configuration. In this sample, it is used to demonstrate communication with SPI3W.

.. _spi3w_polling_configuration_en:

Configurations
==============
1. The following macro can be configured to modify the pin definitions.

   .. code-block:: c

    /* SPI3W Pin Configuration */
    #define SPI3W_CLK_PIN               P4_0
    #define SPI3W_DATA_PIN              P4_1
    #define SPI3W_CS_PIN                P4_2

Building and Downloading
========================
For building and downloading, please refer to the :ref:`Building and Downloading <general_build_download_en>`.


Experimental Verification
==========================

1. After the EVB starts, SPI3W begins communication with the mouse sensor. Once the communication ends, it prints the obtained ID information. If the ID is 0x3E and 0x01 (only for PWM3610DM-SUDU sensor), it means the sample is successful.
   :: 
     SPI3W Read ID: id[0] = 0x3e, id[1] = 0x1
     SPI3W Read ID Pass


Code Overview
=======================
This section introduces the code and process description for initialization and corresponding function implementation in the sample.

Source Code Directory
----------------------

The directory for project file and source code are as follows:

* Project directory: :file:`sdk\\sample\\peripheral\\spi3w\\polling\\proj`
* Source code directory: :file:`sdk\\sample\\peripheral\\spi3w\\polling\\src`


Initialization
---------------
The initialization flow for peripherals can refer to :ref:`Initialization Flow <general_peripheral_init_flow_en>` in :Doc:`General Introduction <../../../../doc/general_introduction/text_en/README>`.

1. Call :cpp:any:`Pad_Config` and :cpp:any:`Pinmux_Config` to configure the PAD and PINMUX of the corresponding pins.

   .. code-block:: c

      void board_spi3w_init(void)
      {
          Pad_Config(SPI3W_CLK_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE,
                     PAD_OUT_HIGH);
          Pad_Config(SPI3W_DATA_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE,
                     PAD_OUT_HIGH);
          Pad_Config(SPI3W_CS_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE,
                     PAD_OUT_HIGH);
      
          Pinmux_Config(SPI3W_CLK_PIN, SPI3W_CLK_MASTER);
          Pinmux_Config(SPI3W_DATA_PIN, SPI3W_DATA_MASTER);
          Pinmux_Config(SPI3W_CS_PIN, SPI3W_CS_MASTER);
      }

2. Call :cpp:any:`RCC_ClockCmd` to enable the SPI3W clock.
3. Initialize the SPI3W peripheral:

   a. Define the :cpp:any:`SPI3W_InitTypeDef` type ``SPI3W_InitStruct``, and call :cpp:any:`SPI3W_StructInit` to pre-fill ``SPI3W_InitStruct`` with default values.
   b. Modify the ``SPI3W_InitStruct`` parameters as needed. The SPI3W initialization parameter configuration is shown in the table below.
   c. Call :cpp:any:`SPI3W_Init` to initialize the SPI3W peripheral.

.. csv-table:: SPI3W Initialization Parameters
  :header: SPI3W Hardware Parameters, Setting in the ``SPI3W_InitStruct`` , SPI3W
  :widths: 40 40 40
  :align: center

  SPI3W Source Clock, :cpp:any:`SPI3W_InitTypeDef::SPI3W_SysClock`, 20000000
  SPI3W Clock, :cpp:any:`SPI3W_InitTypeDef::SPI3W_Speed`, 800000
  SPI3W Mode (3-Wire or 2-Wire), :cpp:any:`SPI3W_InitTypeDef::SPI3W_Mode`, :cpp:any:`SPI3W_3WIRE_MODE`
  Read Delay Cycle, :cpp:any:`SPI3W_InitTypeDef::SPI3W_ReadDelay`, 0x3


.. _spi3w_polling_function_en:

Functional Implementation
--------------------------
The process of reading data in polling mode for SPI3W is shown in the diagram:

.. figure:: ../../../doc/figures/spi3w_polling_read_flow.*
   :align: center
   :scale: 100%
   :alt: Here should be SPI polling read flow
   :name: figure-SPI polling read flow

   SPI3W polling read mode flow


1. Call the ``spi3w_read_byte`` function, passing in the address parameter to read the mouse ID information. The sample uses the ``SPI3W_WAIT_WHILE`` macro to handle status waiting and timeouts.

   a. Wait for the busy status flag to clear.
   b. Call :cpp:any:`SPI3W_ClearRxDataLen` to clear the received data length of SPI3W.
   c. Call :cpp:any:`SPI3W_StartRead`, passing in the address information and the length of data to be read, to start reading the data at that address. Wait for the busy status flag to clear.
   d. Continuously check if data has been received. If data is received, call :cpp:any:`SPI3W_ReadBuf` to read the data from the SPI3W receive FIFO.
   e. Return the read data.

   .. code-block:: c

    uint8_t spi3w_read_byte(uint8_t address)
    {
        uint8_t reg_value = 0xFF;

        /* Check busy before reading */
        SPI3W_WAIT_WHILE(SPI3W_GetFlagStatus(SPI3W_FLAG_BUSY) == SET);

        /* Clear RX data length before reading */
        SPI3W_ClearRxDataLen();

        /* Start Read */
        SPI3W_StartRead(address, 1);

        /* Check read command is write succesfully */
        SPI3W_WAIT_WHILE(SPI3W_GetFlagStatus(SPI3W_FLAG_BUSY) == SET);

        /* Check RX FIFO has received data */
        SPI3W_WAIT_WHILE(SPI3W_GetRxDataLen() == 0);

        /* Read data from RX FIFO */
        SPI3W_ReadBuf(&reg_value, 1);

        return reg_value;
    }


.. _doxygen-group-list-section:

See Also
==========

Please refer to the relevant API Reference:

- :ref:`RCC <group___r_c_c>`
- :ref:`SPI3W <group___s_p_i3_w>`
- :ref:`PINMUX <group___p_i_n_m_u_x>`
