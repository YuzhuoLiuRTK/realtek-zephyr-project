===================================
TIMER Timeout Trigger SPI Transfer
===================================
This example demonstrates how to use the :term:`RAP` (Real Autonomous Peripheral) mechanism to achieve peripheral linkage and cascaded triggering.

The workflow is as follows:
1. **TIMER triggers SPI**: A TIMER timeout event is used as the source to trigger the SPI Master to start a data transfer (Wrap Mode) via RAP.
2. **SPI triggers GPIO**: The "Start" and "End" events of the SPI transfer are used to trigger a GPIO pin toggle via RAP.

This mechanism showcases the powerful event routing capabilities of RAP:
*   **Cascading**: Timer -> SPI -> GPIO.
*   **Many-to-One Mapping**: Both the SPI Start event and End event are routed to the same RAP channel to trigger the same GPIO toggle action. This allows the GPIO output waveform to precisely envelope the SPI transmission frame.

Requirements
=============
For requirements, please refer to the :ref:`Requirements <general_requirements_en>`.

Wiring
==============
This example is configured in SPI Master mode. It is recommended to use a logic analyzer to observe the timing, or short MISO and MOSI for a loopback test.

* **SPI Interface**:
    * **SCK**: P0_1
    * **MOSI**: P0_2
    * **MISO**: P0_4 (Connect to MOSI for loopback test)
    * **CS**: P0_5
* **GPIO Indicator**:
    * **Output**: P0_0 (Used to observe the RAP-triggered toggle signal)
* **GND**: Ensure common ground.

Configurations
==============
1. The TIMER trigger period can be configured via the following macro.

   .. code-block:: c

    #define TIMER_PERIOD                (40000000) /* 1 Second */

2. The Data Frame Length for SPI Wrap Mode can be configured.

   .. code-block:: c

    #define SPI_WRAP_NDF_LEN            8   /* Transfer 8 data frames */

3. The GPIO output pin can be configured.

   .. code-block:: c

    #define OUTPUT_PIN                  P0_0

Building and Downloading
========================
For building and downloading, please refer to the :ref:`Building and Downloading <general_build_download_en>`.

Experimental Verification
==========================
1. Connect the pins as described in the "Wiring" section. A logic analyzer capturing SPI pins and P0_0 simultaneously is recommended.
2. Start the EVB.
3. Observe the results:
   
   * **Timing**: An SPI data transfer occurs every 1 second.
   * **GPIO (P0_0)**: 
     * Toggles once when the SPI transfer starts (around CS falling edge).
     * Toggles again when the SPI transfer ends (around CS rising edge).
     * Effectively, the P0_0 level changes "frame" the SPI transmission process.
   * **Log**: If a loopback connection (MOSI-MISO) is made, the serial port will print the received data.
     ::
       Start RAP timer timeout trigger SPI transfer sample
       SPI_MASTER_Handler
       SPI Master RX Length: 8, Data[0]: ...

Code Overview
=======================
This section introduces the code and process description for initialization and corresponding function implementation in the sample.

Source Code Directory
----------------------

The directory for project file and source code are as follows:

* Project directory: :file:`sdk\\sample\\peripheral\\rap\\timer_trigger_spi\\proj`
* Source code directory: :file:`sdk\\sample\\peripheral\\rap\\timer_trigger_spi\\src`

Initialization
--------------
1. **SPI Master Init**: 
   * Configure as Full Duplex Master mode.
   * Enable **Wrap Mode** (``SPI_WrapModeEn = ENABLE``), which is key for automated RAP transfers.
   * Pre-fill the TX FIFO and set Wrap Mode transfer parameters.

   .. code-block:: c

    /* Configure SPI RAP Action Transfer parameters */
    /* Params: SPIx, CmdLength, WaitCount, TransferLength */
    SPI_SetActionTransfer(SPI_MASTER, 1, 10, 7);
    
    /* Pre-fill TX FIFO & Set NDF */
    SPI_SendBuffer(SPI_MASTER, spi_master_tx_buffer, 1);
    SPI_WrapModeSetTxNdf(SPI_MASTER, SPI_WRAP_NDF_LEN);

2. **GPIO & Timer Init**: Standard configuration, ensuring clocks are enabled.

Functional Implementation
--------------------------
The RAP configuration is implemented in the ``main`` function, using two RAP channels to establish the cascade:

1. **Channel 0 (Timer -> SPI)**:
   * **Source Event**: TIMER Timeout (``TIMER_EVENT_TIMEOUT``).
   * **Action**: SPI Start Transfer (:cpp:any:`SPI_ACTION_START`).

2. **Channel 1 (SPI -> GPIO)**:
   * **Source Events**: SPI Transfer Start (:cpp:any:`SPI_EVENT_START`) **OR** SPI Transfer End (:cpp:any:`SPI_EVENT_END`).
   * **Action**: GPIO Pin Toggle (``GPIO_ACTION_TOGGLE``).
   * **Note**: This demonstrates RAP's "OR" logic, where multiple events routed to the same channel trigger the same action.

   .. code-block:: c

    /* Channel 0: Timer Timeout triggers SPI Start */
    RAP_EventRouteSet(TIMER_EVENT_TIMEOUT, channel0);
    RAP_ActionBindSet(SPI_ACTION_START, channel0);

    /* Channel 1: SPI Start OR SPI End triggers GPIO Toggle */
    RAP_EventRouteSet(SPI_EVENT_START, channel1);
    RAP_EventRouteSet(SPI_EVENT_END, channel1);
    RAP_ActionBindSet(GPIO_ACTION_TOGGLE, channel1);

    /* Enable RAP Mode */
    TIMER_RAPModeCmd(TIMER_NUM, ENABLE);
    SPI_RAPModeCmd(SPI_MASTER, ENABLE);
    GPIO_RAPModeCmd(GPIOA, GPIO_OUT_PIN, ENABLE);

.. _doxygen-group-list-section:

See Also
==========

Please refer to the relevant API Reference:

- :ref:`RAP <group___r_a_p>`
- :ref:`TIMER <group___t_i_m_e_r>`
- :ref:`SPI <group___s_p_i>`
- :ref:`GPIO <group___g_p_i_o>`
- :ref:`RCC <group___r_c_c>`
- :ref:`PINMUX <group___p_i_n_m_u_x>`
- :ref:`NVIC <group___n_v_i_c>`
