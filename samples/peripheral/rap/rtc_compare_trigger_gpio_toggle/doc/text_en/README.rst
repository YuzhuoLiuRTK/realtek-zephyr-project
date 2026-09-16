================================
RTC Compare Trigger GPIO Toggle
================================
This example demonstrates how to use the :term:`RAP` (Real Autonomous Peripheral) mechanism to trigger a GPIO output toggle via an RTC Compare Match event, without CPU intervention.

In this example, the RTC is configured to generate a compare match event at regular intervals, which directly triggers a level toggle on pin P0_0 via the RAP channel. It also demonstrates three mechanisms for reloading the RTC compare value.

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

2. The following macros can be configured to select the RTC compare value reload mode (select one of three).

   .. code-block:: c

    #define SAMPLE_CONFIG_RTC_USING_RELOAD_BY_AUTO_RELOAD         1
    #define SAMPLE_CONFIG_RTC_USING_RELOAD_BY_ACTION_RELOAD       0
    #define SAMPLE_CONFIG_RTC_USING_RELOAD_BY_SHORTCUT_RELOAD     0

   * **AUTO_RELOAD**: The RTC hardware automatically loads the preset value immediately upon a compare match.
   * **ACTION_RELOAD**: The compare event triggers a specific reload Action via RAP to execute the manual reload.
   * **SHORTCUT_RELOAD**: The compare event directly triggers the internal reload task (Shortcut) without CPU or RAP intervention.

3. The following macros can be configured to modify RTC timing parameters.

   .. code-block:: c

    #define RTC_PSC_VALUE                   (3200 - 1)  /* 100ms per tick */
    #define RTC_COMP_VALUE                  (20)        /* Timeout: 2.0s */
    #define RTC_COMP_RELOAD_VALUE           (10)        /* Reload: 1.0s */

Building and Downloading
========================
For building and downloading, please refer to the :ref:`Building and Downloading <general_build_download_en>`.

Experimental Verification
==========================
1. Start the EVB, and the RTC begins running.
2. Observe P0_0 using a logic analyzer:
   
   * Initially, when the RTC counter reaches 2.0s (COMP_VALUE), it triggers P0_0 to toggle.
   * Subsequently, the RTC reloads 1.0s (RELOAD_VALUE), and P0_0 toggles every 1.0s thereafter.

3. (Optional) If interrupts are enabled in the code, the serial port will print RTC compare interrupt information.

Code Overview
=======================
This section introduces the code and process description for initialization and corresponding function implementation in the sample.

Source Code Directory
----------------------

The directory for project file and source code are as follows:

* Project directory: :file:`sdk\\sample\\peripheral\\rap\\rtc_trigger\\proj`
* Source code directory: :file:`sdk\\sample\\peripheral\\rap\\rtc_trigger\\src`

Initialization
--------------
1. Call ``board_gpio_init`` and ``driver_gpio_init`` to initialize P0_0 as output mode.
2. Call ``driver_rtc_init`` to initialize the RTC:
   
   * Configure the prescaler ``RTC_PSC_VALUE`` to set the Tick frequency.
   * Configure the compare value ``RTC_COMP_VALUE`` and reload value ``RTC_COMP_RELOAD_VALUE``.
   * Reset the RTC counter.

   .. code-block:: c

    static void driver_rtc_init(void)
    {
        /* Enable RTC clock and DeInit */
        RCC_ClockCmd(RTC_CLOCK, ENABLE);
        RTC_DeInit();

        /* Configure RTC prescaler */
        RTC_SetPrescaler(RTC_PSC_VALUE);

        /* Configure RTC compare and reload value */
        RTC_SetCompValue(RTC_COMP_NUM, RTC_COMP_VALUE);
        RTC_SetCompReloadValue(RTC_COMP_NUM, RTC_COMP_RELOAD_VALUE);

        /* Reset the RTC counter */
        RTC_ResetCounter();
        
        /* ... Optional Interrupt Configuration ... */
    }

Functional Implementation
--------------------------
The RAP configuration and triggering process are implemented in the ``main`` function:

1. Call :cpp:any:`RAP_ChannelAllocate` to allocate a RAP channel.
2. Call :cpp:any:`RAP_EventRouteSet` to route the RTC compare event to the RAP channel.
3. Call :cpp:any:`RAP_ActionBindSet` to bind the GPIO output toggle action to the RAP channel.
4. Configure the RTC reload mechanism (Auto/Action/Shortcut) based on macro definitions.
5. Enable RAP mode for RTC and GPIO, and start the RTC.

   .. code-block:: c

    int main(void)
    {
        /* ... Initialization ... */

        /* Route RTC Compare event and Bind GPIO Toggle action */
        RAP_EventRouteSet(RTC_EVENT_COMPARE, channel0);
        RAP_ActionBindSet(GPIO_OUT_ACTION_TOGGLE, channel0);

        /* Configure RTC Comparator Reload Mechanism */
    #if (SAMPLE_CONFIG_RTC_USING_RELOAD_BY_AUTO_RELOAD == 1)
        RTC_CompAutoReloadCmd(RTC_COMP_NUM, ENABLE);
    #elif (SAMPLE_CONFIG_RTC_USING_RELOAD_BY_ACTION_RELOAD == 1)
        RAP_ActionBindSet(RTC_ACTION_RELOAD, channel0);
    #elif (SAMPLE_CONFIG_RTC_USING_RELOAD_BY_SHORTCUT_RELOAD == 1)
        RTC_ShortcutCmd(RTC_SHORTCUT_ACTION_RELOAD, RTC_SHORTCUT_EVENT_COMPARE, ENABLE);
    #endif

        /* Enable RAP Mode */
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
