==================================
TIMER Timeout Trigger GPIO Toggle
==================================
This example demonstrates how to use the :term:`RAP` (Real Autonomous Peripheral) mechanism to trigger a GPIO output toggle via the TIMER Timeout event, without CPU intervention.

Additionally, this example showcases three ways to implement TIMER **One-Shot** functionality:
1. **Continuous Mode (Default)**: TIMER auto-reloads and periodically triggers GPIO toggling.
2. **Action Stop Mode**: Stops the TIMER upon timeout by binding the Stop Action via RAP.
3. **Shortcut Stop Mode**: Stops the TIMER directly upon timeout using the peripheral's internal Shortcut mechanism.

Requirements
=============
For requirements, please refer to the :ref:`Requirements <general_requirements_en>`.

Wiring
==============
* Connect **P0_0** to a logic analyzer or oscilloscope (to observe the RAP trigger result).

Configurations
==============
1. The following macros can be configured to select the TIMER operation mode (Continuous or One-Shot).

   .. code-block:: c

    /* 
     * Mode Selection:
     * Both 0: Continuous Mode (Periodic toggling).
     * ...ACTION_STOP = 1: One-Shot using RAP Action.
     * ...SHORTCUT_STOP = 1: One-Shot using Internal Shortcut.
     */
    #define SAMPLE_CONFIG_TIMER_USING_ONE_SHOT_BY_ACTION_STOP     0
    #define SAMPLE_CONFIG_TIMER_USING_ONE_SHOT_BY_SHORTCUT_STOP   0

2. The following macros can be configured to modify the TIMER period.

   .. code-block:: c

    /* Default is 1 second (40MHz clock / 40,000,000) */
    #define TIMER_PERIOD                    (40000000)

3. The following macros can be configured to modify the GPIO output pin.

   .. code-block:: c

    #define OUTPUT_PIN                      P0_0

Building and Downloading
========================
For building and downloading, please refer to the :ref:`Building and Downloading <general_build_download_en>`.

Experimental Verification
==========================
1. Start the EVB.
2. Observe the output waveform of P0_0.

**Case A: Default Configuration (Continuous Mode)**
   * Both One-Shot macros are set to 0.
   * P0_0 toggles its level every 1 second.
   * The output signal is a square wave with a 2-second period and 50% duty cycle.

**Case B: One-Shot Configuration (Action or Shortcut Mode)**
   * Set one of the One-Shot macros to 1.
   * P0_0 toggles its level once after 1 second, and then remains static (TIMER stops running).

Code Overview
=======================
This section introduces the code and process description for initialization and corresponding function implementation in the sample.

Source Code Directory
----------------------

The directory for project file and source code are as follows:

* Project directory: :file:`sdk\\sample\\peripheral\\rap\\timer_timeout_trigger\\proj`
* Source code directory: :file:`sdk\\sample\\peripheral\\rap\\timer_timeout_trigger\\src`

Initialization
--------------
1. Initialize GPIO (P0_0) as output mode.
2. Initialize TIMER1, configured in User-Defined Auto-Reload mode (:cpp:any:`TIMER_MODE_USERDEFINE_AUTO`) with a 1-second period.

Functional Implementation
--------------------------
The RAP configuration is implemented in the ``main`` function:

1. **Basic Configuration**:
   
   * Allocate a RAP channel.
   * Route the TIMER Timeout event (``TIMER_EVENT_TIMEOUT``) to the RAP channel.
   * Bind the GPIO Toggle action (``GPIO_OUT_ACTION_TOGGLE``) to this channel.

2. **One-Shot Stop Mechanism (Optional)**:
   
   * **Shortcut Method**: Enable the internal TIMER Shortcut to link the Event and Action directly, without consuming a RAP action slot.
   * **RAP Action Method**: Bind the TIMER Stop action (``TIMER_ACTION_STOP``) to the same RAP channel. When the timeout occurs, it triggers both the GPIO toggle and the TIMER stop simultaneously.

   .. code-block:: c

    /* Route TIMER Timeout Event to RAP channel */
    RAP_EventRouteSet(TIMER_EVENT_TIMEOUT, channel0);
    /* Bind GPIO Toggle Action to RAP channel */
    RAP_ActionBindSet(GPIO_OUT_ACTION_TOGGLE, channel0);

    /* Configure TIMER One Shot Mode Logic */
    #if (SAMPLE_CONFIG_TIMER_USING_ONE_SHOT_BY_SHORTCUT_STOP == 1)
        /* Method 1: Internal Shortcut */
        TIMER_ShortcutCmd(TIMER_NUM, TIMER_SHORTCUT_ACTION, TIMER_SHORTCUT_EVENT, ENABLE);

    #elif (SAMPLE_CONFIG_TIMER_USING_ONE_SHOT_BY_ACTION_STOP == 1)
        /* Method 2: RAP Action Bind */
        RAP_ActionBindSet(TIMER_ACTION_STOP, channel0);
    #endif

    /* Start TIMER */
    TIMER_ActionTrigger(TIMER_NUM, TIMER_ACTION_START);

.. _doxygen-group-list-section:

See Also
==========

Please refer to the relevant API Reference:

- :ref:`RAP <group___r_a_p>`
- :ref:`TIMER <group___t_i_m_e_r>`
- :ref:`GPIO <group___g_p_i_o>`
- :ref:`RCC <group___r_c_c>`
- :ref:`PINMUX <group___p_i_n_m_u_x>`
- :ref:`NVIC <group___n_v_i_c>`
