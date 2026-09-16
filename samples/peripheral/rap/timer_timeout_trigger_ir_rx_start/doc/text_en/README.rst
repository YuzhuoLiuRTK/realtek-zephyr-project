==================================
TIMER Timeout Trigger IR RX START
==================================
This example demonstrates how to use the :term:`RAP` (Real Autonomous Peripheral) mechanism to synchronously trigger the following actions via a TIMER timeout event, achieving IR signal transmission and reception testing without CPU intervention:

1. **Start PWM Output**: Acts as the IR transmitter source (Carrier Waveform).
2. **Start IR Reception**: Begins capturing the IR signal.
3. **Start DMA Transfer**: Automatically moves received IR data to memory.

Subsequently, another TIMER timeout event automatically stops the PWM output via RAP, simulating the end of the IR signal. The CPU only needs to process data when the IR reception is complete (Idle Timeout).

Key Workflow:

1. **Trigger Start (TIMER1_CH1 Timeout)**:

   * Start PWM output (10kHz, 50% duty cycle).
   * Start IR receiver.
   * Start DMA channel for data transfer.
   * Start Stop Timer (TIMER1_CH2).

2. **Auto Stop (TIMER1_CH2 Timeout)**:

   * After 1 second, stop the PWM output.

3. **Data Processing (IR Idle Interrupt)**:

   * When the IR signal stops, the IR module detects an idle timeout and triggers an interrupt.
   * The CPU reads the remaining data and prints the received waveform data.

Requirements
=============
For requirements, please refer to the :ref:`Requirements <general_requirements_en>`.

Wiring
==============
This example performs a loopback test on the same EVB. Connect as follows:

* **Signal Connection**: Connect **P0_1** (PWM Output / TX Simulator) to **P0_0** (IR Input / RX).
* **GND**: Ensure common ground.

Configurations
==============
1. The following macros can be configured to modify PWM (Transmitter) parameters.

   .. code-block:: c

    #define PWM_OUT_PIN                         P0_1
    #define PWM_OUT_PERIOD                      4000  /* 10kHz @ 40MHz */
    #define PWM_OUT_HIGH_COUNT                  2000  /* 50% Duty Cycle */

2. The following macros can be configured to modify IR RX parameters.

   .. code-block:: c

    #define IR_RX_PIN                           P0_0
    #define IR_RX_DMA_WATERLEVEL                4     /* FIFO Level to trigger DMA */

3. The following macros can be configured to modify the transmission duration.

   .. code-block:: c

    /* Stop Carrier Timer Period: 1 Second */
    #define TIMER_STOP_CARRIER_PERIOD           40000000

Building and Downloading
========================
For building and downloading, please refer to the :ref:`Building and Downloading <general_build_download_en>`.

Experimental Verification
==========================
1. Connect the pins as described in the "Wiring" section.
2. Start the EVB.
3. Observe the serial log. The system will trigger an IR transmission/reception process and print the received data:
   
   * **Log Output**:
     ::
       Start RAP timer timeout trigger IR RX start sample
       ...
       IR_INT_RX_CNT_THR (IR RX Complete)
       IR RX Length: ...
       IR RX Data[0]: 0x...
       IR RX Data[1]: 0x...

Code Overview
=======================
This section introduces the code and process description for initialization and corresponding function implementation in the sample.

Source Code Directory
----------------------

The directory for project file and source code are as follows:

* Project directory: :file:`sdk\\sample\\peripheral\\rap\\timer_trigger_ir_dma\\proj`
* Source code directory: :file:`sdk\\sample\\peripheral\\rap\\timer_trigger_ir_dma\\src`

Initialization
--------------
1. **PWM Init**: Configure TIMER1_CH0 in PWM mode, outputting a 10kHz square wave.
2. **IR Init**: Configure in RX mode, enable DMA requests, and set Idle Timeout threshold.
3. **DMA Init**: Configure DMA channel to read data from IR FIFO to memory buffer.
4. **TIMER Init**:
   * **Start Carrier Timer**: Used to generate the start trigger signal.
   * **Stop Carrier Timer**: Used to generate the stop trigger signal after a 1-second delay.

Functional Implementation
--------------------------
The RAP configuration is implemented in the ``main`` function, using two RAP channels:

1. **Channel 0 (Start Sequence)**:
   * **Source Event**: Start Carrier Timer Timeout.
   * **Actions**: 
     1. Start Stop Carrier Timer (Begin 1-second countdown).
     2. Start PWM Output.
     3. Start IR Reception.
     4. Start DMA Transfer.

2. **Channel 1 (Stop Sequence)**:
   * **Source Event**: Stop Carrier Timer Timeout.
   * **Actions**:
     1. Stop PWM Output (Simulate end of signal).
     2. Stop Stop Carrier Timer itself.

   .. code-block:: c

    /* Channel 0: Start Sequence */
    RAP_EventRouteSet(TIMER_START_CARRIER_EVENT_TIMEOUT, channel0);
    RAP_ActionBindSet(TIMER_STOP_CARRIER_ACTION_START, channel0);
    RAP_ActionBindSet(PWM_OUT_ACTION_START, channel0);
    RAP_ActionBindSet(RAP_ACTION_IR_START_RX, channel0);
    RAP_ActionBindSet(IR_RX_DMA_ACTION_TRANSFER, channel0);

    /* Channel 1: Stop Sequence */
    RAP_EventRouteSet(TIMER_STOP_CARRIER_EVENT_TIMEOUT, channel1);
    RAP_ActionBindSet(TIMER_STOP_CARRIER_ACTION_STOP, channel1);
    RAP_ActionBindSet(PWM_OUT_ACTION_STOP, channel1);

.. _doxygen-group-list-section:

See Also
==========

Please refer to the relevant API Reference:

- :ref:`RAP <group___r_a_p>`
- :ref:`TIMER <group___t_i_m_e_r>`
- :ref:`IR <group___i_r>`
- :ref:`DMA <group___d_m_a>`
- :ref:`RCC <group___r_c_c>`
- :ref:`PINMUX <group___p_i_n_m_u_x>`
- :ref:`NVIC <group___n_v_i_c>`
