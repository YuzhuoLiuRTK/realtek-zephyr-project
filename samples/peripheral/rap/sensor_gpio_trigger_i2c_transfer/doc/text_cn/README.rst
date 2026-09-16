=================================
Sensor GPIO Trigger I2C Transfer
=================================
该示例演示了如何构建一个低功耗的传感器数据采集系统。它利用 :term:`RAP` (Real Autonomous Peripheral) 机制，实现传感器（LIS3DH 加速度计）主动触发 MCU 进行数据搬运的功能，全程无需 CPU 介入。

**工作原理：**
1. **传感器 FIFO**: LIS3DH 被配置为 FIFO Stream 模式。当内部缓存的数据达到阈值（Watermark）时，传感器通过 INT1 引脚输出高电平。
2. **RAP 触发 I2C**: MCU 的 GPIO 捕捉到 INT1 的信号，通过 RAP 自动触发 I2C 控制器启动读取传输。
3. **RAP 触发 DMA**: I2C 的“开始传输”事件再次通过 RAP 触发 DMA 通道开启。
4. **自动搬运**: DMA 将 I2C FIFO 中的加速度数据搬运至内存 Buffer。
5. **CPU 唤醒**: 当 DMA 完成指定数量的数据块搬运（Block Transfer）后，产生中断唤醒 CPU 进行数据处理。

这种机制极大地降低了系统功耗和 CPU 负载，非常适合穿戴设备或物联网传感器节点。

环境需求
========
该示例的环境需求，可参考 :ref:`环境需求 <general_requirements_cn>`。

硬件连线
==============
本示例需要连接 LIS3DH 加速度传感器模块。

* **I2C 接口**:
    * **SCL**: P0_4
    * **SDA**: P0_2
    * **GND/VCC**: 连接至开发板对应电源引脚
* **中断引脚**:
    * **INT1 (Sensor)** -> **P0_6 (MCU Input)**: 传感器的数据就绪中断信号。

.. note::
   请确保 LIS3DH 的 I2C 地址选择引脚配置正确，本示例默认使用地址 ``0x18``。

流程图
======
.. mermaid::

   flowchart TD
      A[LIS3DH Sensor] -- Sampling --> B(Sensor FIFO Full)
      B -- INT1 Pin High --> C[MCU GPIO P0_6]
      C -- RAP Event --> D[I2C Controller]
      D -- RAP Event --> E[DMA Controller]
      E -- Transfer Data --> F[RAM Buffer]
      F -- Block Count Done --> G((CPU Interrupt))

配置选项
==============
1. **采样配置**:
   可在 ``main.c`` 中修改每次触发读取的样本数量。

   .. code-block:: c

    /* 3 sets of X/Y/Z data per trigger */
    #define TRIGGER_SAMPLE_NUM          (3)
    #define BYTES_PER_SAMPLE            (6)

2. **DMA 块计数**:
   修改触发多少次中断后唤醒 CPU。

   .. code-block:: c

    #define DMA_BLOCK_COUNTER_NUM       (10)

3. **I2C 引脚**:
   若使用不同引脚，请修改宏定义。

   .. code-block:: c

    #define I2C_MASTER_SCL_PIN          P0_4
    #define I2C_MASTER_SDA_PIN          P0_2

编译和下载
==========
该示例的编译和下载流程，可参考 :ref:`编译和下载 <general_build_download_cn>`。

测试验证
==========================
1. 按照“硬件连线”章节连接 LIS3DH 模块。
2. 编译并下载固件。
3. 晃动传感器或静置，观察串口日志。
4. **现象**:
   * CPU 处于空闲状态（在示例中为 while(1) 循环）。
   * 当传感器积累了足够数据（触发 10 次 FIFO 阈值中断）后，DMA 中断触发。
   * 串口打印出一批 X, Y, Z 轴的原始数据。

   ::

     Start Sensor gpio trigger i2c transfer sample
     Sensor ID: 0x33
     DMA Block Counter Interrupt
     data0: x: 120, y: -45, z: 1024
     data1: x: 125, y: -40, z: 1020
     ...

代码介绍
=======================
该章节主要介绍示例中的初始化和 RAP 级联配置。

源码路径
--------
* 源码路径: :file:`sdk\\sample\\peripheral\\rap\\gpio_trigger_i2c_dma\\src`

初始化
------
1. **LIS3DH 初始化**:
   * 配置 ODR 为 50Hz。
   * 开启 FIFO Stream 模式，设置阈值（Threshold）为 18 字节（3组数据）。
   * 配置 INT1 引脚在 FIFO 达到阈值时输出高电平。

2. **I2C 预配置 (Wrapper Mode)**:
   * 为了配合 RAP 自动传输，I2C 控制器需要预先配置好“读哪里”和“读多少”。
   * 代码中设置了从 ``LIS3DH_REG_OUT_X_L`` 开始读取，利用地址自动递增特性。

   .. code-block:: c

    /* Tell I2C to read starting from OUT_X_L with Auto-Increment */
    uint8_t cmd = LIS3DH_REG_OUT_X_L | LIS3DH_I2C_MS_BIT;
    lis3dh_config_burst_read(I2C_MASTER, cmd, TRIGGER_SAMPLE_NUM * BYTES_PER_SAMPLE);

功能实现 (RAP 级联)
--------------------
RAP 建立了 **GPIO -> I2C -> DMA** 的级联触发关系：

1. **通道 0**: GPIO 输入事件触发 I2C 开始传输。
2. **通道 1**: I2C 开始传输事件触发 DMA 通道使能。

.. code-block:: c

    /* GPIO in event trigger I2C transfer start */
    RAP_EventRouteSet(GPIO_IN_EVENT_IN, channel0);
    RAP_ActionBindSet(I2C_MASTER_ACTION_TRANSFER_START, channel0);

    /* I2C transfer start event trigger DMA start */
    RAP_EventRouteSet(I2C_MASTER_EVENT_TRANSFER_START, channel1);
    RAP_ActionBindSet(I2C_MASTER_RX_DMA_ACTION_START, channel1);

DMA 块传输
----------
DMA 被配置为 **Block Counter** 模式。这意味着 RAP 会触发多次 DMA 传输，但只有当传输次数达到 ``DMA_BLOCK_COUNTER_NUM`` (10次) 时，DMA 才会产生中断请求 CPU 处理。这有效地实现了数据的批处理。

.. _doxygen-group-list-section:

See Also
==========

相关 API Reference 请查看：

- :ref:`RAP <group___r_a_p>`
- :ref:`I2C <group___i_2_c>`
- :ref:`DMA <group___d_m_a>`
- :ref:`GPIO <group___g_p_i_o>`
- :ref:`RCC <group___r_c_c>`
- :ref:`PINMUX <group___p_i_n_m_u_x>`
- :ref:`NVIC <group___n_v_i_c>`
