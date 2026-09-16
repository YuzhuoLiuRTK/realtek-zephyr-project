==================================
TIMER Timeout Trigger IR TX Start
==================================
This example demonstrates how to use the :term:`RAP` (Real Autonomous Peripheral) mechanism to trigger the IR module's transmission action (TX) via a TIMER timeout event. It utilizes DMA to automatically transfer pre-generated NEC protocol waveform data, achieving periodic IR signal transmission without CPU intervention.

In this example, the CPU pre-calculates and fills the NEC protocol waveform data into a memory buffer. The TIMER generates a trigger signal every 1 second, which starts the IR transmission via RAP. During transmission, the IR module automatically fetches subsequent waveform data via DMA requests.

Key Workflow:
1. **Waveform Generation**: The CPU calculates NEC protocol waveform data (Leader, Data, Stop) and stores it in a memory buffer.
2. **Pre-configuration**: Configure IR in TX mode with DMA requests enabled; configure DMA to transfer data from memory to the IR TX FIFO.
3. **Trigger**: The TIMER generates a timeout event every 1 second, triggering the IR to start transmission via RAP.
4. **Transmission**: The IR module consumes FIFO data and outputs the waveform, simultaneously triggering DMA to refill data.
5. **Completion**: When all data is sent, the DMA generates a Transfer Done interrupt.

Requirements
=============
For requirements, please refer to the :ref:`Requirements <general_requirements_en>`.

Wiring
==============
This example demonstrates IR transmission. Connect an oscilloscope or an IR receiver for observation:

* **Signal Output**: Connect **P0_0** (IR TX) to an oscilloscope probe or the data pin of an IR receiver module.
* **GND**: Ensure common ground.

Configurations
==============
1. The following macros can be configured to modify the IR carrier frequency.

   .. code-block:: c

    /* IR Carrier Frequency: 38kHz */
    IR_InitStruct.IR_Freq = 38000;

2. The following macros can be configured to modify the TIMER trigger period.

   .. code-block:: c

    #define TIMER_PERIOD                40000000 /* 1 Second @ 40MHz */

3. The NEC data payload can be modified in the ``main`` function.

   .. code-block:: c

    uint32_t nec_payload = 0x00F720DF;

Building and Downloading
========================
For building and downloading, please refer to the :ref:`Building and Downloading <general_build_download_en>`.

Experimental Verification
==========================
1. Connect the devices as described in the "Wiring" section.
2. Start the EVB.
3. Observe the results:
   
   * **Oscilloscope**: Every 1 second, a burst of 38kHz carrier-modulated NEC protocol waveform can be observed on pin P0_0.
   * **Serial Log**: A log is printed every time transmission completes (DMA transfer done).
     ::
       Start RAP timer timeout trigger ir tx sample
       IR_TX_DMA_Handler: IR TX Done
       IR_TX_DMA_Handler: IR TX Done
       ...

Code Overview
=======================
This section introduces the code and process description for initialization and corresponding function implementation in the sample.

Source Code Directory
----------------------

The directory for project file and source code are as follows:

* Project directory: :file:`sdk\\sample\\peripheral\\rap\\timer_trigger_ir_tx\\proj`
* Source code directory: :file:`sdk\\sample\\peripheral\\rap\\timer_trigger_ir_tx\\src`

Initialization
--------------
1. **IR Init**: Configure in TX mode, set carrier frequency to 38kHz, and enable DMA function.
2. **DMA Init**: Configure DMA channel to transfer data from memory buffer (``ir_tx_data_buffer``) to ``IR->IR_TX_FIFO``.
3. **Waveform Generation**: The ``ir_generate_nec_waveform`` function is responsible for converting 32-bit NEC data into the waveform encoding format (Mark/Space duration) required by the IR hardware.

   .. code-block:: c

    /* Generate NEC Waveform */
    uint32_t ir_tx_data_length = ir_generate_nec_waveform(nec_payload, ir_tx_data_buffer);

Functional Implementation
--------------------------
The RAP configuration is implemented in the ``main`` function:

1. **RAP Routing**:
   
   * Route the TIMER Timeout event (``TIMER_EVENT_TIMEOUT``) to a RAP channel.
   * Bind the IR Start TX action (:c:macro:`RAP_ACTION_IR_START_TX`) to this channel.

2. **Start**:
   * Enable RAP mode for TIMER and IR.
   * Enable the DMA channel to be ready for transfer.
   * Start the TIMER. IR transmission will subsequently be triggered by hardware timing.

   .. code-block:: c

    /* Route TIMER Timeout Event to RAP channel */
    RAP_EventRouteSet(TIMER_EVENT_TIMEOUT, channel0);
    /* Bind IR START TX Action to RAP channel */
    RAP_ActionBindSet(RAP_ACTION_IR_START_TX, channel0);

    /* Enable RAP Mode */
    TIMER_RAPModeCmd(TIMER_NUM, ENABLE);
    IR_RAPModeCmd(ENABLE);

    /* Start TIMER loop */
    TIMER_ActionTrigger(TIMER_NUM, TIMER_ACTION_START);

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
