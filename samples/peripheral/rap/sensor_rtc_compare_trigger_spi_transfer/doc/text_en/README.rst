================================
Sensor RTC Trigger SPI Transfer
================================
This example demonstrates how to build a fully automated **periodic sensor sampling system**.

Unlike using GPIO interrupts for triggering, this example uses the :term:`RAP` (Real Autonomous Peripheral) combined with the **RTC (Real-Time Clock)** to periodically trigger the SPI interface to read sensor data and use DMA to move the data to memory. The CPU can remain in sleep mode throughout the process until a specified number of data blocks have been collected.

**Working Principle:**
1. **RTC Periodic Trigger**: The RTC is configured to generate a Compare Match event every 1 second.
2. **RAP Triggers SPI**: The RTC event triggers the SPI Master to start a transfer via RAP. The SPI operates in Wrap Mode, automatically sending the preset read command.
3. **RAP Triggers DMA**: The SPI "Transfer Start" event cascades to trigger the DMA channel enablement.
4. **Data Transfer**: The DMA automatically moves the data received by the SPI (including the Dummy Byte and sensor data) into the buffer.
5. **Batch Processing Wake-up**: When 10 data blocks are collected (i.e., after 10 seconds), the DMA block transfer interrupt wakes up the CPU for data processing.

This mechanism is ideal for applications like environmental monitoring or motion logging that do not require real-time processing but need periodic logging.

Requirements
=============
For requirements, please refer to the :ref:`Requirements <general_requirements_en>`.

Wiring
==============
This example is configured in SPI Master mode connected to a LIS3DH module. Since the trigger is internal (RTC), **connecting the sensor's interrupt pin is not required**.

* **SPI Interface**:
    * **SCK**: P0_4
    * **MOSI**: P0_2
    * **MISO**: P0_1
    * **CS**: P0_0
* **GND/VCC**: Connect to the corresponding power pins on the EVB.

Flowchart
=========
.. mermaid::

   flowchart TD
      A[RTC Timer] -- 1 Sec Interval --> B(Compare Event)
      B -- RAP Event --> C[SPI Master Start]
      C -- RAP Event --> D[DMA Channel Enable]
      C -- Read Command --> E[LIS3DH Sensor]
      E -- Sensor Data --> C
      D -- Transfer RX Data --> F[RAM Buffer]
      F -- Block Count (10) Done --> G((CPU Interrupt))

Configurations
==============
1. **Sampling Interval**:
   Configure the sampling interval via RTC prescaler and reload values. Currently set to 1 second.

   .. code-block:: c

    #define RTC_PRESCALER_VALUE         (3200 - 1) /* 100ms Tick */
    #define RTC_COMP_VALUE              10         /* Initial Trigger: 1s */
    #define RTC_COMP_RELOAD_VALUE       10         /* Interval: 1s */

2. **DMA Batch Size**:
   Configure how many samples to collect before waking the CPU.

   .. code-block:: c

    #define DMA_BLOCK_COUNTER_NUM       (10)

3. **SPI Sample Size**:
   The amount of data read per trigger (1 set of X/Y/Z data).

   .. code-block:: c

    #define TRIGGER_SAMPLE_NUM          (1)

Building and Downloading
========================
For building and downloading, please refer to the :ref:`Building and Downloading <general_build_download_en>`.

Experimental Verification
==========================
1. Connect the SPI hardware as described in the "Wiring" section.
2. Build and download the firmware.
3. Observe the serial log:

   * System initializes and displays Sensor ID.
   * **Wait for approximately 10 seconds** (since RTC triggers once per second and DMA waits for 10 triggers).
   * The serial port prints 10 sets of data collected over the past 10 seconds at once.
      ::
        Start Sensor RTC trigger spi transfer sample
        Sensor ID: 0x33
        dma rx handler
        DMA Block Counter Interrupt
        data0: x: 104, y: -50, z: 980
        data1: x: 102, y: -48, z: 982
        ...

Code Overview
=======================
Source Code Directory
----------------------
* Source code directory: :file:`sdk\\sample\\peripheral\\rap\\rtc_trigger_spi_dma\\src`

Core Logic
------------
1. **RTC Configuration**:
   The RTC uses :cpp:any:`RTC_EnableCompAutoReload` to configure auto-reload mode, ensuring the continuity of the time base without software intervention to reset the counter.

2. **RAP Cascade**:
   Establishes the **RTC -> SPI -> DMA** trigger chain.
   
   .. code-block:: c

    /* Channel 0: RTC Compare -> SPI Start */
    RAP_EventRouteSet(RTC_EVENT_COMPARE, channel0);
    RAP_ActionBindSet(SPI_MASTER_ACTION_TRANSFER_START, channel0);

    /* Channel 1: SPI Start -> DMA Enable */
    RAP_EventRouteSet(SPI_MASTER_EVENT_TRANSFER_START, channel1);
    RAP_ActionBindSet(SPI_MASTER_RX_DMA_ACTION_START, channel1);

3. **Data Offset Handling**:
   Since SPI is full-duplex, every read operation requires sending a 1-byte command. This results in the first byte of each data set in the receive buffer being invalid (Dummy Byte). The code skips this byte using an offset when printing the log.

   .. code-block:: c

    /* Block Size = 1 (Dummy) + 6 (Data) */
    uint32_t block_offset = j * (1 + BYTES_PER_SAMPLE * TRIGGER_SAMPLE_NUM);
    /* Real Data starts at block_offset + 1 */

.. _doxygen-group-list-section:

See Also
==========

Please refer to the relevant API Reference:

- :ref:`RAP <group___r_a_p>`
- :ref:`RTC <group___r_t_c>`
- :ref:`SPI <group___s_p_i>`
- :ref:`DMA <group___d_m_a>`
- :ref:`RCC <group___r_c_c>`
- :ref:`PINMUX <group___p_i_n_m_u_x>`
- :ref:`NVIC <group___n_v_i_c>`
