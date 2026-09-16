====================================
TIMER Timeout Trigger SPI3W Transfer
====================================
This example demonstrates how to use the :term:`RAP` (Real Autonomous Peripheral) mechanism to periodically trigger a Quick Burst Read operation on the SPI3W (3-Wire SPI) interface via a TIMER, and cascade the SPI3W transfer completion event to trigger a GPIO toggle.

The workflow is as follows:
1. **TIMER triggers SPI3W**: The TIMER generates a timeout event every 1 second, triggering the SPI3W to start a Quick Burst Read sequence via RAP.
2. **SPI3W triggers GPIO**: Upon completion of the SPI3W read, an end event is generated, which triggers a GPIO pin toggle via RAP, serving as a transfer completion indicator.

This mechanism is well-suited for low-power applications requiring periodic sensor data reading, eliminating the need for frequent CPU intervention during the startup process.

Requirements
=============
For requirements, please refer to the :ref:`Requirements <general_requirements_en>`.

Wiring
==============
This example uses the SPI3W interface (commonly used for connecting optical sensors). A logic analyzer is recommended for observing timing.

* **SPI3W Interface**:
    * **CLK**: P0_1
    * **DATA**: P0_2
    * **QB (Quick Burst Trigger)**: P0_4 (Outputs the Burst pulse signal in this example)
* **GPIO Indicator**:
    * **Output**: P0_0 (Used to observe the RAP-triggered toggle signal)
* **GND**: Ensure common ground.

Configurations
==============
1. The TIMER trigger period can be configured via the following macro.

   .. code-block:: c

    #define TIMER_PERIOD                (40000000) /* 1 Second @ 40MHz */

2. The SPI3W communication speed and read delay can be configured.

   .. code-block:: c

    #define SPI3W_SPEED                 800000    /* 800kHz */
    #define SPI3W_READ_DELAY            3         /* 2.5us Delay */

3. The data length for Quick Burst Read can be configured.

   .. code-block:: c

    /* Read 3 bytes */
    SPI3W_SetQuickBurstRead(3, ENABLE);

Building and Downloading
========================
For building and downloading, please refer to the :ref:`Building and Downloading <general_build_download_en>`.

Experimental Verification
==========================
1. Connect the pins as described in the "Wiring" section. A logic analyzer is recommended.
2. Start the EVB.
3. Observe the results:
   
   * **Timing**: Every 1 second, the P0_4 (QB) pin outputs a pulse of approximately 5us, followed by SPI3W clock and data transmission.
   * **GPIO (P0_0)**: Toggles its state once after every SPI3W transfer completes.
   * **Log**: If an SPI3W slave device is connected or simulated, the serial port will print the read data.
     ::
       Start RAP timer timeout trigger SPI3W transfer sample
       SPI3W_Handler
       SPI3W RX Length 3, Data[0]: ...

Code Overview
=======================
This section introduces the code and process description for initialization and corresponding function implementation in the sample.

Source Code Directory
----------------------

The directory for project file and source code are as follows:

* Project directory: :file:`sdk\\sample\\peripheral\\rap\\timer_trigger_spi3w\\proj`
* Source code directory: :file:`sdk\\sample\\peripheral\\rap\\timer_trigger_spi3w\\src`

Initialization
--------------
1. **SPI3W Init**: 
   * Configure for 2-Wire mode (Clock + Data), using an extra pin for the Quick Burst trigger signal.
   * Set communication speed to 800kHz.
   * Configure **Quick Burst Read** parameters: Set read length to 3 bytes and pulse width to 19 cycles (approx. 5us).

   .. code-block:: c

    /* Configure Quick Burst Read */
    SPI3W_SetQuickBurstRead(3, ENABLE);
    SPI3W_SetQuickBurstPulseWidth(19);

2. **GPIO & Timer Init**: Configure P0_0 as output and Timer in 1-second auto-reload mode.

Functional Implementation
--------------------------
The RAP configuration is implemented in the ``main`` function, using two RAP channels:

1. **Channel 0 (Timer -> SPI3W)**:
   * **Source Event**: TIMER Timeout (``TIMER_EVENT_TIMEOUT``).
   * **Action**: SPI3W Start (:cpp:any:`SPI3W_ACTION_START`), which triggers the Quick Burst sequence.

2. **Channel 1 (SPI3W -> GPIO)**:
   * **Source Event**: SPI3W Transfer End (``SPI3W_EVENT_END``).
   * **Action**: GPIO Pin Toggle (``GPIO_ACTION_TOGGLE``).

   .. code-block:: c

    /* Channel 0: Timer Timeout triggers SPI3W Start */
    RAP_EventRouteSet(TIMER_EVENT_TIMEOUT, channel0);
    RAP_ActionBindSet(SPI3W_ACTION_START, channel0);

    /* Channel 1: SPI3W End triggers GPIO Toggle */
    RAP_EventRouteSet(SPI3W_EVENT_END, channel1);
    RAP_ActionBindSet(GPIO_ACTION_TOGGLE, channel1);

    /* Enable RAP Mode */
    TIMER_RAPModeCmd(TIMER_NUM, ENABLE);
    SPI_RAPModeCmd(ENABLE);
    GPIO_RAPModeCmd(GPIOA, GPIO_PIN_BIT, ENABLE);

.. _doxygen-group-list-section:

See Also
==========

Please refer to the relevant API Reference:

- :ref:`RAP <group___r_a_p>`
- :ref:`TIMER <group___t_i_m_e_r>`
- :ref:`SPI3W <group___s_p_i3_w>`
- :ref:`GPIO <group___g_p_i_o>`
- :ref:`RCC <group___r_c_c>`
- :ref:`PINMUX <group___p_i_n_m_u_x>`
- :ref:`NVIC <group___n_v_i_c>`
