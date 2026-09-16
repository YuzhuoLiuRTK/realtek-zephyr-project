=================================
Sensor GPIO Trigger SPI Transfer
=================================
This example demonstrates how to implement an automated sensor data acquisition system based on the SPI interface using the :term:`RAP` (Real Autonomous Peripheral) mechanism.

Similar to the I2C version, this example achieves fully automated hardware cascade triggering. However, when using the SPI interface, it handles full-duplex communication data transport and timing coordination.

**Working Principle:**
1. **Sensor Trigger**: The LIS3DH accelerometer asserts the INT1 pin high when its internal FIFO data reaches the threshold.
2. **RAP Triggers SPI**: The MCU's GPIO captures the INT1 rising edge and triggers the SPI Master to start a transfer (Wrap Mode) via RAP.
3. **RAP Triggers DMA**: The SPI "Transfer Start" event triggers the RX DMA channel enablement via RAP.
4. **Data Transfer**: While the SPI Master sends the read command, the DMA automatically moves the received data (including the Dummy Byte from the command phase and subsequent valid data) to memory.
5. **Batch Processing Wake-up**: The CPU is woken up by an interrupt only after the DMA completes a specified number of block transfers.

Requirements
=============
For requirements, please refer to the :ref:`Requirements <general_requirements_en>`.

Wiring
==============
This example is configured in SPI Master mode connected to a LIS3DH module.

* **SPI Interface**:
    * **SCK**: P0_4
    * **MOSI**: P0_2
    * **MISO**: P0_1
    * **CS**: P0_0
* **Interrupt Pin**:
    * **INT1 (Sensor)** -> **P0_6 (MCU Input)**: Sensor data ready signal.

.. note::
   SPI is a full-duplex bus. Ensure the sensor supports SPI mode (LIS3DH usually selects SPI/I2C mode via CS pin state or register configuration).

Flowchart
=========
.. mermaid::

   flowchart TD
      A[LIS3DH Sensor] -- FIFO Watermark --> B(INT1 Pin High)
      B -- GPIO Event --> C[RAP Channel 0]
      C -- Trigger --> D[SPI Master Start]
      D -- Event Start --> E[RAP Channel 1]
      E -- Trigger --> F[DMA Channel Enable]
      D <-- SPI Bus --> A
      F -- Save RX Data --> G[RAM Buffer]
      G -- Block Count Done --> H((CPU Interrupt))

Configurations
==============
1. **Sample Count Configuration**:
   
   .. code-block:: c

    /* Read 10 sets of samples (6 bytes per set) per trigger */
    #define TRIGGER_SAMPLE_NUM          (10)

2. **DMA Block Counter**:
   
   .. code-block:: c

    /* Trigger CPU interrupt only after 10 RAP actions */
    #define DMA_BLOCK_COUNTER_NUM       (10)

3. **SPI Pin Definitions**:

   .. code-block:: c

    #define SPI_MASTER_SCK_PIN          P0_4
    #define SPI_MASTER_MOSI_PIN         P0_2
    #define SPI_MASTER_MISO_PIN         P0_1
    #define SPI_MASTER_CS_PIN           P0_0

Building and Downloading
========================
For building and downloading, please refer to the :ref:`Building and Downloading <general_build_download_en>`.

Experimental Verification
==========================
1. Connect the SPI hardware as described in the "Wiring" section.
2. Build and download the firmware.
3. Observe the serial log:
   
   * System initializes and reads Sensor ID (0x33).
   * CPU enters idle state.
   * When the sensor FIFO fills and triggers enough times, the DMA interrupt occurs, printing the data.
   * **Note**: Since SPI is full-duplex, the first received byte corresponds to the input during the command transmission (usually invalid data or status). The code automatically offsets this byte when printing.

   ::

     Start Sensor gpio trigger spi transfer sample
     Sensor ID: 0x33
     dma rx handler
     DMA Block Counter Interrupt
     data0: x: 104, y: -50, z: 980
     ...

Code Overview
=======================
Source Code Directory
----------------------
* Source code directory: :file:`sdk\\sample\\peripheral\\rap\\gpio_trigger_spi_dma\\src`

Key Implementation Details
---------------------------
1. **SPI Wrap Mode**:
   To support RAP automated triggering, **Wrap Mode** must be enabled during SPI initialization. This mode allows the SPI controller to automatically repeat transfers based on preset TX FIFO content (typically the read command) without CPU refilling the FIFO every time.

   .. code-block:: c

    SPI_InitStruct.SPI_WrapModeEn = ENABLE;
    SPI_InitStruct.SPI_TXNDF      = 8; // Number of Data Frames

2. **RAP Cascade**:
   * **GPIO -> SPI**: GPIO Input event triggers SPI Start.
   * **SPI -> DMA**: SPI Transfer Start event triggers DMA Enable. This ensures DMA operates only when the SPI bus is active.

   .. code-block:: c

    RAP_EventRouteSet(GPIO_IN_EVENT_IN, channel0);
    RAP_ActionBindSet(SPI_MASTER_ACTION_TRANSFER_START, channel0);

    RAP_EventRouteSet(SPI_MASTER_EVENT_TRANSFER_START, channel1);
    RAP_ActionBindSet(SPI_MASTER_RX_DMA_ACTION_START, channel1);

3. **RX Data Handling**:
   The SPI transfer consists of a 1-byte Command Header + N bytes of Data. The DMA buffer size is set to ``(N * Samples) + 1``. When processing data, the code explicitly skips the 0-th byte of each block (the RX data corresponding to the Command Phase).

   .. code-block:: c

    /* Indexing Correction for SPI: Skip 1st byte (Dummy) */
    uint32_t data_offset = block_offset + 1 + i * 6;

.. _doxygen-group-list-section:

See Also
==========

Please refer to the relevant API Reference:

- :ref:`RAP <group___r_a_p>`
- :ref:`SPI <group___s_p_i>`
- :ref:`DMA <group___d_m_a>`
- :ref:`GPIO <group___g_p_i_o>`
- :ref:`RCC <group___r_c_c>`
- :ref:`PINMUX <group___p_i_n_m_u_x>`
- :ref:`NVIC <group___n_v_i_c>`
