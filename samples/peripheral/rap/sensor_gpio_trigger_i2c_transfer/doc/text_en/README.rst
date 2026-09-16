=================================
Sensor GPIO Trigger I2C Transfer
=================================
This example demonstrates how to build a low-power sensor data acquisition system. It uses the :term:`RAP` (Real Autonomous Peripheral) mechanism to enable a sensor (LIS3DH accelerometer) to actively trigger the MCU for data transfer, requiring zero CPU intervention during the acquisition process.

**Working Principle:**
1. **Sensor FIFO**: The LIS3DH is configured in FIFO Stream mode. When the internal data buffer reaches the threshold (Watermark), the sensor asserts the INT1 pin High.
2. **RAP Triggers I2C**: The MCU's GPIO captures the INT1 signal and automatically triggers the I2C controller to start a read transaction via RAP.
3. **RAP Triggers DMA**: The I2C "Transfer Start" event triggers the DMA channel enablement via RAP.
4. **Automatic Transfer**: DMA moves the acceleration data from the I2C FIFO to the RAM buffer.
5. **CPU Wake-up**: The CPU is only woken up by an interrupt after the DMA completes a specified number of block transfers.

This mechanism significantly reduces system power consumption and CPU load, making it ideal for wearable devices or IoT sensor nodes.

Requirements
=============
For requirements, please refer to the :ref:`Requirements <general_requirements_en>`.

Wiring
==============
This example requires a LIS3DH accelerometer module.

* **I2C Interface**:
    * **SCL**: P0_4
    * **SDA**: P0_2
    * **GND/VCC**: Connect to the corresponding power pins on the EVB.
* **Interrupt Pin**:
    * **INT1 (Sensor)** -> **P0_6 (MCU Input)**: Sensor data ready interrupt signal.

.. note::
   Ensure the LIS3DH I2C address selection pin is configured correctly. This example uses address ``0x18`` by default.

Flowchart
=========
.. mermaid::

   flowchart TD
      A[LIS3DH Sensor] -- Sampling --> B(Sensor FIFO Full)
      B -- INT1 Pin High --> C[MCU GPIO P0_6]
      C -- RAP Event --> D[I2C Controller]
      D -- RAP Event --> E[DMA Controller]
      E -- Transfer Data --> F[RAM Buffer]
      F -- Block Count Done --> G((CPU Interrupt))

Configurations
==============
1. **Sampling Configuration**:
   Modify the number of samples read per trigger in ``main.c``.

   .. code-block:: c

    /* 3 sets of X/Y/Z data per trigger */
    #define TRIGGER_SAMPLE_NUM          (3)
    #define BYTES_PER_SAMPLE            (6)

2. **DMA Block Counter**:
   Modify how many triggers occur before waking the CPU.

   .. code-block:: c

    #define DMA_BLOCK_COUNTER_NUM       (10)

3. **I2C Pins**:
   Modify macros if using different pins.

   .. code-block:: c

    #define I2C_MASTER_SCL_PIN          P0_4
    #define I2C_MASTER_SDA_PIN          P0_2

Building and Downloading
========================
For building and downloading, please refer to the :ref:`Building and Downloading <general_build_download_en>`.

Experimental Verification
==========================
1. Connect the LIS3DH module as described in the "Wiring" section.
2. Build and download the firmware.
3. Move the sensor or leave it stationary, and observe the serial log.
4. **Observation**:
   * The CPU remains idle (in the ``while(1)`` loop in the example).
   * Once the sensor accumulates enough data (triggering the FIFO threshold 10 times), the DMA interrupt fires.
   * The serial port prints a batch of raw X, Y, Z axis data.

   ::

     Start Sensor gpio trigger i2c transfer sample
     Sensor ID: 0x33
     DMA Block Counter Interrupt
     data0: x: 120, y: -45, z: 1024
     data1: x: 125, y: -40, z: 1020
     ...

Code Overview
=======================
This section introduces the initialization and RAP cascade configuration in the sample.

Source Code Directory
----------------------
* Source code directory: :file:`sdk\\sample\\peripheral\\rap\\gpio_trigger_i2c_dma\\src`

Initialization
--------------
1. **LIS3DH Init**:
   * ODR configured to 50Hz.
   * FIFO Stream mode enabled with a threshold of 18 bytes (3 data sets).
   * INT1 pin configured to go High when FIFO reaches the threshold.

2. **I2C Pre-configuration (Wrapper Mode)**:
   * To support automated RAP transfers, the I2C controller must be pre-configured with "where to read" and "how much to read".
   * The code sets up reading starting from ``LIS3DH_REG_OUT_X_L`` using the address auto-increment feature.

   .. code-block:: c

    /* Tell I2C to read starting from OUT_X_L with Auto-Increment */
    uint8_t cmd = LIS3DH_REG_OUT_X_L | LIS3DH_I2C_MS_BIT;
    lis3dh_config_burst_read(I2C_MASTER, cmd, TRIGGER_SAMPLE_NUM * BYTES_PER_SAMPLE);

Functional Implementation (RAP Cascade)
----------------------------------------
RAP establishes a **GPIO -> I2C -> DMA** cascade triggering relationship:

1. **Channel 0**: GPIO Input event triggers I2C Start Transfer.
2. **Channel 1**: I2C Start Transfer event triggers DMA Channel Enable.

.. code-block:: c

    /* GPIO in event trigger I2C transfer start */
    RAP_EventRouteSet(GPIO_IN_EVENT_IN, channel0);
    RAP_ActionBindSet(I2C_MASTER_ACTION_TRANSFER_START, channel0);

    /* I2C transfer start event trigger DMA start */
    RAP_EventRouteSet(I2C_MASTER_EVENT_TRANSFER_START, channel1);
    RAP_ActionBindSet(I2C_MASTER_RX_DMA_ACTION_START, channel1);

DMA Block Transfer
------------------
The DMA is configured in **Block Counter** mode. This means RAP will trigger DMA transfers multiple times, but the DMA will only generate an interrupt to the CPU when the transfer count reaches ``DMA_BLOCK_COUNTER_NUM`` (10 times). This effectively implements data batching.

.. _doxygen-group-list-section:

See Also
==========

Please refer to the relevant API Reference:

- :ref:`RAP <group___r_a_p>`
- :ref:`I2C <group___i_2_c>`
- :ref:`DMA <group___d_m_a>`
- :ref:`GPIO <group___g_p_i_o>`
- :ref:`RCC <group___r_c_c>`
- :ref:`PINMUX <group___p_i_n_m_u_x>`
- :ref:`NVIC <group___n_v_i_c>`
