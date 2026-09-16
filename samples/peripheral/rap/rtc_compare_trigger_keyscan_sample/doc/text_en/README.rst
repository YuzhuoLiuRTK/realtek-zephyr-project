=====================================
RTC Compare Trigger KEYSCAN Sample
=====================================
This example demonstrates how to use the :term:`RAP` (Real Autonomous Peripheral) mechanism to trigger KEYSCAN manual scans via RTC periodic compare events, implementing a low-power key detection scheme.

The workflow is as follows:
1. **Initial State**: KEYSCAN is configured in key trigger mode. When a key press is detected, the CPU wakes up and enters the interrupt.
2. **Scanning State**: After the first key press is detected, the system switches to RTC trigger mode. The RTC periodically triggers KEYSCAN via RAP to scan until all keys are released.
3. **End State**: When all keys are detected as released, the RTC and RAP are disabled, and KEYSCAN is reconfigured back to key trigger mode, waiting for the next key event.

Requirements
=============
For requirements, please refer to the :ref:`Requirements <general_requirements_en>`.

Wiring
==============
This example is configured for a 2x2 matrix keyboard, connected as follows:

* **Row**: P0_4, P0_5
* **Column**: P0_6, P0_7

Configurations
==============
1. The following macros can be configured to modify the row and column pins and size of KEYSCAN.

   .. code-block:: c

    #define KEYBOARD_ROW_SIZE               2
    #define KEYBOARD_COLUMN_SIZE            2
    #define KEYBOARD_ROW_0                  P0_4
    /* ... other pins ... */

2. The following macros can be configured to modify the RTC scanning period (debounce/polling interval).

   .. code-block:: c

    #define RTC_PSC_VALUE                   (320 - 1)   /* 10ms tick */
    #define RTC_COMP_VALUE                  (5)         /* 50ms initial wait */
    #define RTC_COMP_RELOAD_VALUE           (5)         /* 50ms interval */

Building and Downloading
========================
For building and downloading, please refer to the :ref:`Building and Downloading <general_build_download_en>`.

Experimental Verification
==========================
1. Start the EVB.
2. Press any key on the matrix keyboard.
3. The serial assistant will output key detection logs:
   ::
     KEYSCAN First Key Press Detected
     KEYSCAN One Key Press Detected: (0, 1) ...
   
4. Hold the key down; logs will output the detection results periodically (triggered by RTC).
5. Release the key; the serial port outputs the release information, and scanning stops:
   ::
     KEYSCAN: All Keys Released

Code Overview
=======================
This section introduces the code and process description for initialization and corresponding function implementation in the sample.

Source Code Directory
----------------------

The directory for project file and source code are as follows:

* Project directory: :file:`sdk\\sample\\peripheral\\rap\\keyscan_trigger\\proj`
* Source code directory: :file:`sdk\\sample\\peripheral\\rap\\keyscan_trigger\\src`

Initialization
--------------
1. Call ``board_keyscan_init`` to initialize keyboard pins.
2. Call ``driver_keyscan_init`` to initialize the KEYSCAN peripheral, with the initial mode set to :cpp:any:`KEYSCAN_MANUAL_SEL_KEY` (Physical Key Trigger).
3. Call ``driver_rtc_init`` to initialize the RTC, setting a 50ms timing period.

   .. code-block:: c

    static void driver_keyscan_init(KEYSCANManualSel_TypeDef Manual_Sel)
    {
        /* ... Clock Enable ... */
        KEYSCAN_InitStruct.KEYSCAN_ScanMode   = KEYSCAN_MANUAL_SCAN_MODE;
        KEYSCAN_InitStruct.KEYSCAN_DetectMode = KEYSCAN_DETECT_MODE_EDGE;
        KEYSCAN_InitStruct.KEYSCAN_ManualSel  = Manual_Sel; /* Key or Register Bit */
        KEYSCAN_Init(KEYSCAN, &KEYSCAN_InitStruct);
        /* ... Interrupt Configuration ... */
    }

Functional Implementation
--------------------------
The logic is mainly divided into RAP binding and state machine switching (handled in the KEYSCAN interrupt):

1. **RAP Binding**: In the ``main`` function, bind the RTC compare event to a RAP channel and associate two actions: trigger KEYSCAN manual scan and RTC self-reload.
   
   .. code-block:: c

    RAP_EventRouteSet(RTC_EVENT_COMPARE, channel0);
    RAP_ActionBindSet(RAP_ACTION_KEYSCAN_MANUAL, channel0);
    RAP_ActionBindSet(RTC_ACTION_RELOAD, channel0);

2. **State Switching (Interrupt Handling)**:
   
   * **First Press**: Upon detecting a key in the interrupt, switch the KEYSCAN trigger source to Register Bit Trigger (:cpp:any:`KEYSCAN_MANUAL_SEL_BIT`) and enable RTC and RAP modes. Periodic scanning via RTC begins.
   
   * **Continuous Press**: The RTC triggers a scan every 50ms. After KEYSCAN completes the scan, it generates an interrupt to read FIFO data.
   
   * **Key Release**: When the FIFO is detected as empty, stop the RTC, disable RAP mode, and re-initialize KEYSCAN to physical key trigger mode, waiting for the next wakeup.

   .. code-block:: c

    void KEYSCAN_Handler(void)
    {
        /* ... Read FIFO ... */
        if (KEYSCAN_GetFlagState(KEYSCAN, KEYSCAN_FLAG_EMPTY) != SET) {
            if (is_first_pressed == false) {
                /* Switch to Periodic Scan Mode via RTC RAP */
                KEYSCAN_SetManualSelect(KEYSCAN, KEYSCAN_MANUAL_SEL_BIT);
                RTC_RAPModeCmd(ENABLE);
                KEYSCAN_RAPModeCmd(KEYSCAN, ENABLE);
                RTC_ActionTrigger(RTC_ACTION_START);
            }
        } else {
            /* All Keys Released: Stop RTC, Disable RAP, Reset to Key Trigger */
            RTC_ActionTrigger(RTC_ACTION_STOP);
            /* ... Disable RAP Modes ... */
            driver_keyscan_init(KEYSCAN_MANUAL_SEL_KEY);
        }
    }

.. _doxygen-group-list-section:

See Also
==========

Please refer to the relevant API Reference:

- :ref:`RAP <group___r_a_p>`
- :ref:`RTC <group___r_t_c>`
- :ref:`KEYSCAN <group___k_e_y_s_c_a_n>`
- :ref:`RCC <group___r_c_c>`
- :ref:`PINMUX <group___p_i_n_m_u_x>`
- :ref:`NVIC <group___n_v_i_c>`
