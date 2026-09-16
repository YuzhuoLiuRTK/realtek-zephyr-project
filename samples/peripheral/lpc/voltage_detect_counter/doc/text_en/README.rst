==================
Voltage Detection
==================
This example verifies the voltage detection function of the :term:`LPC`.

Use P2_2 as the pin for voltage detection. An LPC interrupt is triggered when the input voltage of P2_2 exceeds the set threshold.

.. note::
  - Pins that can be used for voltage detection are P2_0 ~ P2_7, Vbat, with a detection voltage threshold range of 60mV~3600mV

Requirements
=============
For requirements, please refer to the :ref:`Requirements <general_requirements_en>`.
   
Wiring
==============
Connect P2_2 to an external voltage input.

Configurations
==============
1. The following macro can be configured to modify the pin definitions.

   .. code-block:: c

    #define LPC_CAPTURE_PIN             P2_2
    #define LPC_CAPTURE_CHANNEL         LPC_CHANNEL_ADC2

2. The following macros can be configured to modify the trigger edge settings of the LPC comparator.

   .. code-block:: c

    #define LPC_VOLTAGE_DETECT_EDGE         LPC_Vin_Over_Vth        /*< Set this macro to select the LPC detect edge. Selectable parameters include LPC_Vin_Over_Vth and LPC_Vin_Below_Vth. */

3. The following macro can be configured to modify the trigger threshold voltage of the LPC comparator.

   .. code-block:: c

    #define LPC_COMPARE_VOLTAGE              LPC_1080_mV        /*< Configure LPC Threshold Voltage.*/


Building and Downloading
========================
For building and downloading, please refer to the :ref:`Building and Downloading <general_build_download_en>`.


Experimental Verification
==========================

1. After the EVB starts, observe the log in the Debug Analyzer tool.
   ::
     Start lpc voltage detect sample

2. When P2_2 detects an input voltage higher than 1000 mV, it triggers the :c:macro:`LPC_INT_VOLTAGE_DETECT` interrupt, and logs the entry into the voltage detection interrupt in the Debug Analyzer tool.
   ::
     LPC_Handler: Voltage Interrupt Detected!


Code Overview
=======================

This section mainly introduces the code and process description for the initialization and corresponding functional implementation in the example.

Source Code Path
--------

The project files and source code paths are as follows:

* Project directory: :file:`sdk\\sample\\peripheral\\lpc\\voltage_detect\\proj`
* Source code directory: :file:`sdk\\sample\\peripheral\\lpc\\voltage_detect\\src`

Initialization
------
The initialization flow for peripherals can refer to :ref:`Initialization Flow <general_peripheral_init_flow_en>` in :Doc:`General Introduction <../../../../doc/general_introduction/text_en/README>`.

1. Call :cpp:any:`Pad_Config` and :cpp:any:`Pinmux_Config` to configure the PAD and PINMUX for the corresponding pins.

   .. code-block:: c

    void board_lpc_init(void)
    {
        Pad_Config(LPC_TEST_PIN, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE, PAD_OUT_HIGH);
        Pinmux_Config(LPC_TEST_PIN, IDLE_MODE);
    }

2. Since LPC is located in the AON area, there is no need to call :cpp:any:`RCC_ClockCmd`.
3. Initialize the LPC peripheral:

   a. Define a :cpp:any:`LPC_InitTypeDef` type ``LPC_InitStruct`` and call :cpp:any:`LPC_StructInit` to prefill ``LPC_InitStruct`` with default values.
   b. Modify the ``LPC_InitStruct`` parameters according to the requirements. The initialization parameter configuration for LPC is shown in the table below.
   c. Call :cpp:any:`LPC_Init` to initialize the LPC peripheral.

.. csv-table:: LPC Initialization Parameters
  :header: LPC Hardware Parameters, Setting in the ``LPC_InitStruct`` , LPC
  :widths: 40 40 40
  :align: center

  Channel, :cpp:any:`LPC_InitTypeDef::LPC_Channel`, :cpp:any:`LPC_CAPTURE_CHANNEL`
  Edge, :cpp:any:`LPC_InitTypeDef::LPC_Edge`, :cpp:any:`LPC_VOLTAGE_DETECT_EDGE`
  Threshold Voltage, :cpp:any:`LPC_InitTypeDef::LPC_Threshold`, :cpp:any:`LPC_COMPARE_VOLTAGE`

4. Call :cpp:any:`LPC_INTConfig` to configure the LPC compare interrupt :c:macro:`LPC_INT_VOLTAGE_DETECT` and NVIC. For related NVIC configurations, refer to :ref:`Interrupt Configuration<general_nvic_config_en>`.
5. Call :cpp:any:`LPC_Cmd` to enable the LPC comparison.

.. _lpc_voltage_detection_function_en:

Functional Implementation
--------------------------

1. When P2_2 detects a voltage higher than the set threshold of 1000 mV, it triggers the :c:macro:`LPC_INT_VOLTAGE_DETECT` interrupt. In the interrupt function, print relevant information and clear the interrupt flag.

   .. code-block:: c

    void LPC_Handler(void)
    {
       /* Check if Voltage Detect Interrupt occurred */
      if (LPC_GetINTStatus(LPC0, LPC_INT_VOLTAGE_DETECT) == SET)
      {
          DBG_DIRECT("LPC_Handler: Voltage Interrupt Detected!");

          /* Disable the interrupt to prevent continuous triggering if the voltage */
          LPC_INTConfig(LPC0, LPC_INT_VOLTAGE_DETECT, DISABLE);

          /* Clear the interrupt status */
          LPC_ClearINTStatus(LPC0, LPC_INT_VOLTAGE_DETECT);
       }

     }


.. _doxygen-group-list-section:

See Also
==========

Please refer to the relevant API Reference:

- :ref:`LPC <group___l_p_c>`
- :ref:`PINMUX <group___p_i_n_m_u_x>`
- :ref:`RCC <group___r_c_c>`
- :ref:`NVIC <group___n_v_i_c>`