=============================
RTC Tick Trigger GPIO Toggle
=============================
This example demonstrates how to use the :term:`RAP` (Real Autonomous Peripheral) mechanism to trigger a GPIO output toggle via the RTC Tick event (periodic tick signal), without CPU intervention.

In this example, the RTC is configured to generate a periodic Tick signal (default is 100ms). Whenever an RTC Tick event occurs, it directly triggers the P0_0 pin to toggle its level via the RAP channel, thereby generating a square wave signal.

Requirements
=============
For requirements, please refer to the :ref:`Requirements <general_requirements_en>`.

Wiring
==============
* Connect P0_0 to a logic analyzer or oscilloscope (for observing output waveforms).

Configurations
==============
1. The following macros can be configured to modify the GPIO output pin.

   .. code-block:: c

    #define OUTPUT_PIN                      P0_0

2. The following macros can be configured to modify the RTC Tick frequency.

   .. code-block:: c

    /* 
     * Formula: Tick_Freq = Clock_Src (32kHz) / (PSC + 1)
     * Value (3200 - 1) generates a 10Hz Tick (100ms interval)
     */
    #define RTC_PSC_VALUE                   (3200 - 1)

Building and Downloading
========================
For building and downloading, please refer to the :ref:`Building and Downloading <general_build_download_en>`.

Experimental Verification
==========================
1. Start the EVB, and the RTC begins running.
2. Observe P0_0 using a logic analyzer.
3. P0_0 will output a square wave with a 50% duty cycle.
   
   * The level toggles every 100ms (based on default Tick configuration).
   * The signal period is 200ms (5Hz).

4. (Optional) If ``#if 0`` in the interrupt configuration section of the code is changed to ``#if 1``, the serial port will print the following log upon each Tick:
   ::
     RTC_Handler: RTC_INT_TICK

Code Overview
=======================
This section introduces the code and process description for initialization and corresponding function implementation in the sample.

Source Code Directory
----------------------

The directory for project file and source code are as follows:

* Project directory: :file:`sdk\\sample\\peripheral\\rap\\rtc_tick_trigger\\proj`
* Source code directory: :file:`sdk\\sample\\peripheral\\rap\\rtc_tick_trigger\\src`

Initialization
--------------
1. Call ``board_gpio_init`` and ``driver_gpio_init`` to initialize P0_0 as output mode.
2. Call ``driver_rtc_init`` to initialize the RTC:
   
   * Configure the prescaler ``RTC_PSC_VALUE`` to set the time interval for Tick generation.
   * Reset the RTC counter.

   .. code-block:: c

    static void driver_rtc_init(void)
    {
        /* Enable RTC clock and DeInit */
        RCC_ClockCmd(RTC_CLOCK, ENABLE);
        RTC_DeInit();

        /* Configure RTC prescaler to generate the tick */
        RTC_SetPrescaler(RTC_PSC_VALUE);

        /* Reset the RTC counter */
        RTC_ResetCounter();
        
        /* ... Optional Interrupt Configuration ... */
    }

Functional Implementation
--------------------------
The RAP configuration and triggering process are implemented in the ``main`` function:

1. Call :cpp:any:`RAP_ChannelAllocate` to allocate a RAP channel.
2. Call :cpp:any:`RAP_EventRouteSet` to route the RTC Tick event (:c:macro:`RAP_EVENT_RTC_TICK`) to the allocated RAP channel.
3. Call :cpp:any:`RAP_ActionBindSet` to bind the GPIO output toggle action (``RAP_ACTION_GPIOA_DRTOGGLE(0)``) to the RAP channel.
4. Enable RAP mode for RTC and GPIO.
5. Start the RTC.

   .. code-block:: c

    int main(void)
    {
        /* ... Initialization ... */

        /* Configure RAP Channel */
        uint8_t channel0;
        RAP_ChannelAllocate(&channel0);

        /* Route RTC Tick Event to RAP channel */
        RAP_EventRouteSet(RTC_EVENT_TICK, channel0);
        /* Bind GPIO Toggle Action to RAP channel */
        RAP_ActionBindSet(GPIO_OUT_ACTION_TOGGLE, channel0);

        /* Enable RAP Mode for RTC and GPIO */
        RTC_RAPModeCmd(ENABLE);
        GPIO_RAPModeCmd(GPIO_OUT_PORT, GPIO_OUT_PIN, ENABLE);

        /* Start RTC */
        RTC_ActionTrigger(RTC_ACTION_START);

        while (1) { }
    }

.. _doxygen-group-list-section:

See Also
==========

Please refer to the relevant API Reference:

- :ref:`RAP <group___r_a_p>`
- :ref:`RTC <group___r_t_c>`
- :ref:`GPIO <group___g_p_i_o>`
- :ref:`RCC <group___r_c_c>`
- :ref:`PINMUX <group___p_i_n_m_u_x>`
- :ref:`NVIC <group___n_v_i_c>`
