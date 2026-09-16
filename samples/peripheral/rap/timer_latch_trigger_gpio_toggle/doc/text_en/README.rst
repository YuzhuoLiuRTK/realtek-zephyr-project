===============================
TIMER Latch Trigger GPIO Toggle
===============================
This example demonstrates how to use the :term:`RAP` (Real Autonomous Peripheral) mechanism to trigger a GPIO output toggle via the TIMER Latch FIFO threshold event, without CPU intervention.

In this example, the TIMER is configured in Latch Mode to capture pulses from an input signal. When the number of captured pulses reaches a preset threshold (default is 4), a threshold event is generated, which directly triggers the P0_0 pin level toggle via the RAP channel. It also demonstrates how to read latched data via ISR or DMA.

Requirements
=============
For requirements, please refer to the :ref:`Requirements <general_requirements_en>`.

Wiring
==============
This example involves signal generation (simulation) and signal capture. The following hardware connections are required:

* **Signal Loop**: Connect **P0_2** (Simulated Pulse Output) to **P0_1** (TIMER Latch Input).
* **Observation Pin**: Connect **P0_0** to a logic analyzer or oscilloscope (to observe the RAP trigger result).

Configurations
==============
1. The following macros can be configured to select the data retrieval mode for the Latch FIFO (select one of two).

   .. code-block:: c

    #define SAMPLE_CONFIG_TIMER_LATCH_MODE_ISR       1
    #define SAMPLE_CONFIG_TIMER_LATCH_MODE_DMA       0

   * **ISR Mode**: The threshold event triggers an interrupt, and the CPU reads the FIFO data in the ISR.
   * **DMA Mode**: The threshold event triggers a DMA request, and the DMA controller automatically transfers FIFO data to memory.

2. The following macros can be configured to modify the trigger threshold and pins.

   .. code-block:: c

    #define TIMER_LATCH_TRIGGER_THRESHOLD   4       /* Pulse count required to trigger RAP event */
    #define TIMER_LATCH_TRIGGER_PAD         P0_1    /* Capture input pin */

Building and Downloading
========================
For building and downloading, please refer to the :ref:`Building and Downloading <general_build_download_en>`.

Experimental Verification
==========================
1. Connect P0_2 to P0_1 as described in the "Wiring" section.
2. Start the EVB.
3. After the code runs, P0_2 will automatically generate 4 pulses.
4. Observe P0_0 and the serial logs:
   
   * **Waveform**: When the TIMER detects the rising edge of the 4th pulse on P0_1, the P0_0 level toggles.
   * **Log**: The serial port prints the latched data read (via ISR or DMA mode).
     ::
       TIMER_Handler
       timer_latch_data[0] = 0x...
       ...

Code Overview
=======================
This section introduces the code and process description for initialization and corresponding function implementation in the sample.

Source Code Directory
----------------------

The directory for project file and source code are as follows:

* Project directory: :file:`sdk\\sample\\peripheral\\rap\\timer_latch_trigger\\proj`
* Source code directory: :file:`sdk\\sample\\peripheral\\rap\\timer_latch_trigger\\src`

Initialization
--------------
1. Call ``driver_timer_init`` to initialize the TIMER:
   
   * Configure as :cpp:any:`TIMER_MODE_FREERUN`.
   * Enable Latch function, setting the trigger edge, threshold (``TIMER_LATCH_TRIGGER_THRESHOLD``), and input pin.
   * Enable DMA request or NVIC interrupt based on the configuration.

   .. code-block:: c

    static void driver_timer_init(void)
    {
        /* ... Basic Config ... */
        TIMER_InitStruct.TIMER_Latch.TIMER_LatchEn[0] = ENABLE;
        TIMER_InitStruct.TIMER_Latch.TIMER_LatchTriggerMode[0] = TIMER_LATCH_TRIGGER_RISING_EDGE;
        TIMER_InitStruct.TIMER_Latch.TIMER_LatchThreshold = TIMER_LATCH_TRIGGER_THRESHOLD;
        TIMER_InitStruct.TIMER_Latch.TIMER_LatchTriggerPad = TIMER_LATCH_TRIGGER_PAD;
        
        /* ... DMA or NVIC Config ... */
        TIMER_TimeBaseInit(TIMER_NUM, &TIMER_InitStruct);
    }

Functional Implementation
--------------------------
The RAP configuration and pulse simulation are implemented in the ``main`` function:

1. **RAP Configuration**:
   
   * Allocate a RAP channel.
   * Route the TIMER Latch Threshold event (``TIMER_EVENT_LATCH_THRESHOLD``) to the RAP channel.
   * Bind the GPIO toggle action to this channel.

   .. code-block:: c

    RAP_EventRouteSet(TIMER_EVENT_LATCH_THRESHOLD, channel0);
    RAP_ActionBindSet(GPIO_OUT_ACTION_TOGGLE, channel0);
    TIMER_RAPModeCmd(TIMER_NUM, ENABLE);
    GPIO_RAPModeCmd(GPIO_OUT_PORT, GPIO_OUT_PIN, ENABLE);

2. **Trigger Source Simulation**:
   
   * After starting the TIMER, call the ``pad_generate_pulse`` function to generate a specified number of pulses on P0_2 to simulate an external input signal.

   .. code-block:: c

    /* Start TIMER */
    TIMER_ActionTrigger(TIMER_NUM, TIMER_ACTION_START);

    /* Simulate pulse input */
    pad_generate_pulse(P0_2, TIMER_LATCH_TRIGGER_THRESHOLD);

.. _doxygen-group-list-section:

See Also
==========

Please refer to the relevant API Reference:

- :ref:`RAP <group___r_a_p>`
- :ref:`TIMER <group___t_i_m_e_r>`
- :ref:`DMA <group___d_m_a>`
- :ref:`GPIO <group___g_p_i_o>`
- :ref:`RCC <group___r_c_c>`
- :ref:`PINMUX <group___p_i_n_m_u_x>`
- :ref:`NVIC <group___n_v_i_c>`
