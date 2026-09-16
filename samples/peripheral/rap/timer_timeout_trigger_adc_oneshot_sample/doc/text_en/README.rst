=========================================
TIMER Timeout Trigger ADC Oneshot Sample
=========================================
This example demonstrates how to use the :term:`RAP` (Real Autonomous Peripheral) mechanism to implement cascaded triggering between peripherals without CPU intervention.

The workflow is as follows:
1. **TIMER Trigger**: TIMER1 periodically (every 1 second) generates a timeout event.
2. **ADC Sample**: The TIMER timeout event triggers the ADC to perform a One-Shot Sample via RAP Channel 0.
3. **GPIO Toggle**: When the ADC sampling is complete (Done event), it triggers the GPIO output to toggle via RAP Channel 1.

This mechanism is ideal for applications requiring precise timing sampling while minimizing CPU load.

Requirements
=============
For requirements, please refer to the :ref:`Requirements <general_requirements_en>`.

Wiring
==============
* **Analog Input**: Connect **P2_0** to a voltage source to be measured (e.g., 1.5V or the middle pin of a potentiometer).
* **Observation Pin**: Connect **P0_0** to a logic analyzer or oscilloscope (to observe the RAP cascaded trigger result).

Configurations
==============
1. The following macros can be configured to modify the TIMER period.

   .. code-block:: c

    /* Default is 1 second (40MHz clock / 40,000,000) */
    #define TIMER_PERIOD                    (40000000)

2. The following macros can be configured to modify the ADC input pin and channel.

   .. code-block:: c

    #define ADC_PIN                         P2_0
    #define ADC_CHANNEL                     ADC_Channel_Index_0

3. The following macros can be configured to modify the GPIO output pin.

   .. code-block:: c

    #define OUTPUT_PIN                      P0_0

Building and Downloading
========================
For building and downloading, please refer to the :ref:`Building and Downloading <general_build_download_en>`.

Experimental Verification
==========================
1. Connect the devices as described in the "Wiring" section.
2. Start the EVB.
3. Observe the output waveform of P0_0:
   
   * P0_0 toggles its level every 1 second (triggered by the TIMER -> ADC -> GPIO chain).
   * The output signal is a square wave with a 2-second period and 50% duty cycle.

4. (Optional) If ADC interrupt is enabled (``#if 1``), the serial port will print the sampled data:
   ::
     ADC_Handler
     ADC_INT_ONE_SHOT_DONE -> sample_data: ...

Code Overview
=======================
This section introduces the code and process description for initialization and corresponding function implementation in the sample.

Source Code Directory
----------------------

The directory for project file and source code are as follows:

* Project directory: :file:`sdk\\sample\\peripheral\\rap\\timer_trigger_adc\\proj`
* Source code directory: :file:`sdk\\sample\\peripheral\\rap\\timer_trigger_adc\\src`

Initialization
--------------
1. Initialize GPIO (P0_0) as output mode.
2. Initialize TIMER1, configured in User-Defined Auto-Reload mode (:cpp:any:`TIMER_MODE_USERDEFINE_AUTO`) with a 1-second period.
3. Initialize ADC (P2_0), configured in One-Shot mode with Schedule Index 0 enabled.

Functional Implementation
--------------------------
The RAP cascaded configuration is implemented in the ``main`` function, using two RAP channels:

1. **Channel 0 (Timer -> ADC)**:
   
   * Route the TIMER Timeout event (``TIMER_EVENT_TIMEOUT``) to Channel 0.
   * Bind the ADC Sample action (``ADC_ACTION_SAMPLE``) to Channel 0.

2. **Channel 1 (ADC -> GPIO)**:
   
   * Route the ADC Done event (``ADC_EVENT_DONE``) to Channel 1.
   * Bind the GPIO Toggle action (``GPIO_OUT_ACTION_TOGGLE``) to Channel 1.

3. **Start**:
   * Enable RAP mode for all peripherals.
   * Start the TIMER to begin the periodic triggering process.

   .. code-block:: c

    /* ... Initialization ... */
    
    /* Route TIMER Timeout Event to RAP channel0 to trigger ADC */
    RAP_EventRouteSet(TIMER_EVENT_TIMEOUT, channel0);
    RAP_ActionBindSet(ADC_ACTION_SAMPLE, channel0);

    /* Route ADC Done Event to RAP channel1 to trigger GPIO */
    RAP_EventRouteSet(ADC_EVENT_DONE, channel1);
    RAP_ActionBindSet(GPIO_OUT_ACTION_TOGGLE, channel1);

    /* Enable RAP Mode */
    TIMER_RAPModeCmd(TIMER_NUM, ENABLE);
    ADC_RAPModeCmd(ADC, ENABLE);
    GPIO_RAPModeCmd(GPIO_OUT_PORT, GPIO_OUT_PIN, ENABLE);

    /* Start TIMER */
    TIMER_ActionTrigger(TIMER_NUM, TIMER_ACTION_START);

.. _doxygen-group-list-section:

See Also
==========

Please refer to the relevant API Reference:

- :ref:`RAP <group___r_a_p>`
- :ref:`TIMER <group___t_i_m_e_r>`
- :ref:`ADC <group___a_d_c>`
- :ref:`GPIO <group___g_p_i_o>`
- :ref:`RCC <group___r_c_c>`
- :ref:`PINMUX <group___p_i_n_m_u_x>`
- :ref:`NVIC <group___n_v_i_c>`
