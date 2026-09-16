=============================
GPIO In Trigger GPIO Toggle
=============================
This example demonstrates how to use the :term:`RAP` (Real Autonomous Peripheral) mechanism to trigger a GPIO output toggle directly via a GPIO input event, without CPU intervention.

In this example, P0_1 is configured as an input pin and P0_0 as an output pin. When P0_1 detects a falling edge signal, it directly triggers P0_0 to toggle its level via the RAP channel. Additionally, P0_2 is used in the code to simulate pulse signal generation for P0_1 input.

Requirements
=============
For requirements, please refer to the :ref:`Requirements <general_requirements_en>`.

Wiring
==============
* Connect P0_2 to P0_1 (for simulating input signals).
* Connect P0_0 to a logic analyzer or oscilloscope (for observing output effects).

Configurations
==============
1. The following macros can be configured to modify the GPIO input and output pins.

   .. code-block:: c

    #define OUTPUT_PIN                      P0_0
    #define INPUT_PIN                       P0_1

2. The following macros can be configured to modify the RAP event and action mapping (must correspond to the pins).

   .. code-block:: c

    #define GPIO_OUT_ACTION_TOGGLE          RAP_ACTION_GPIOA_DRTOGGLE(0)
    #define GPIO_IN_EVENT_IN                RAP_EVENT_GPIOA(1)

Building and Downloading
========================
For building and downloading, please refer to the :ref:`Building and Downloading <general_build_download_en>`.

Experimental Verification
==========================
1. Start the EVB. The code will automatically generate pulse signals on pin P0_2.
2. Since P0_2 is connected to P0_1, P0_1 will detect the edge signals.
3. Observe P0_0 using a logic analyzer. You can see that when a falling edge occurs on P0_1, the level of P0_0 toggles.
4. (Optional) If ``#if 0`` in the interrupt configuration section of the code is changed to ``#if 1``, the serial port will print the following log upon each trigger:
   ::
     Enter GPIO_Pin_Handler success

Code Overview
=======================
This section introduces the code and process description for initialization and corresponding function implementation in the sample.

Source Code Directory
----------------------

The directory for project file and source code are as follows:

* Project directory: :file:`sdk\\sample\\peripheral\\rap\\gpio_trigger\\proj`
* Source code directory: :file:`sdk\\sample\\peripheral\\rap\\gpio_trigger\\src`

Initialization
--------------
1. Call ``board_gpio_init`` to configure the PAD and PINMUX for P0_0 (Output), P0_1 (Input), and P0_2 (Simulation Output).
2. Call ``driver_gpio_init`` to initialize the GPIO peripheral:
   
   * Configure P0_0 as output mode.
   * Configure P0_1 as input mode, enable debounce function, and set the trigger method to edge trigger (active low).

   .. code-block:: c

    static void driver_gpio_init(void)
    {
        /* Enable GPIO clock */
        RCC_ClockCmd(GPIOA_CLOCK, ENABLE);

        /* Configure GPIO parameters as output mode (P0_0) */
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_StructInit(&GPIO_InitStruct);
        GPIO_InitStruct.GPIO_Pin        = GPIO_OUT_PIN;
        GPIO_InitStruct.GPIO_Dir        = GPIO_DIR_OUT;
        GPIO_InitStruct.GPIO_INTEventEn = DISABLE;
        GPIO_Init(GPIO_OUT_PORT, &GPIO_InitStruct);

        /* Configure GPIO parameters as input mode (P0_1) */
        GPIO_InitStruct.GPIO_Pin        = GPIO_IN_PIN;
        GPIO_InitStruct.GPIO_Dir        = GPIO_DIR_IN;
        GPIO_InitStruct.GPIO_INTEventEn = ENABLE;
        GPIO_InitStruct.GPIO_Trigger    = GPIO_TRIGGER_EDGE;
        GPIO_InitStruct.GPIO_Polarity   = GPIO_POLARITY_ACTIVE_LOW;
        
        /* Configure GPIO Debounce parameters */
        GPIO_InitStruct.GPIO_DebounceEn    = ENABLE;
        /* ... Debounce settings ... */
        
        GPIO_Init(GPIO_IN_PORT, &GPIO_InitStruct);
    }

Functional Implementation
--------------------------
The RAP configuration and triggering process are implemented in the ``main`` function:

1. Call :cpp:any:`RAP_ChannelAllocate` to allocate a RAP channel.
2. Call :cpp:any:`RAP_EventRouteSet` to route the GPIO input event (P0_1) to the allocated RAP channel.
3. Call :cpp:any:`RAP_ActionBindSet` to bind the GPIO output toggle action (P0_0) to the RAP channel.
4. Call :cpp:any:`GPIO_RAPModeCmd` to enable the RAP mode for GPIO.
5. Call ``pad_generate_pulse`` to generate pulses on P0_2 to simulate external input signals.

   .. code-block:: c

    int main(void)
    {
        /* ... Initialization ... */

        /* Configure RAP channel */
        uint8_t channel0;
        RAP_ChannelAllocate(&channel0);

        /* Route GPIO IN Event to RAP channel */
        RAP_EventRouteSet(GPIO_IN_EVENT_IN, channel0);
        /* Bind GPIO Toggle Action to RAP channel */
        RAP_ActionBindSet(GPIO_OUT_ACTION_TOGGLE, channel0);

        /* Enable GPIO RAP Mode */
        GPIO_RAPModeCmd(GPIO_OUT_PORT, GPIO_OUT_PIN, ENABLE);
        GPIO_RAPModeCmd(GPIO_IN_PORT, GPIO_IN_PIN, ENABLE);

        /* Simulate pulse input using P0_2 */
        pad_generate_pulse(P0_2, 2);

        while (1)
        {
        }
    }

.. _doxygen-group-list-section:

See Also
==========

Please refer to the relevant API Reference:

- :ref:`RAP <group___r_a_p>`
- :ref:`GPIO <group___g_p_i_o>`
- :ref:`RCC <group___r_c_c>`
- :ref:`PINMUX <group___p_i_n_m_u_x>`
- :ref:`NVIC <group___n_v_i_c>`
