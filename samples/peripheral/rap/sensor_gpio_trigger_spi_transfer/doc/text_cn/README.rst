=================================
Sensor GPIO Trigger SPI Transfer
=================================
该示例演示了如何利用 :term:`RAP` (Real Autonomous Peripheral) 机制，实现基于 SPI 接口的传感器数据自动采集系统。

与 I2C 版本类似，本示例实现了全自动的硬件级联触发，但在使用 SPI 接口时，涉及到全双工通信的数据搬运和时序配合。

**工作原理：**
1. **传感器触发**: LIS3DH 加速度计在 FIFO 数据达到阈值时，拉高 INT1 引脚。
2. **RAP 触发 SPI**: MCU 的 GPIO 捕捉到 INT1 上升沿，通过 RAP 触发 SPI Master 启动传输 (Wrap Mode)。
3. **RAP 触发 DMA**: SPI 的“开始传输”事件通过 RAP 触发 RX DMA 通道开启。
4. **数据搬运**: SPI Master 发送读取命令的同时，DMA 自动将接收到的数据（包含命令阶段的 Dummy Byte 和随后的有效数据）搬运至内存。
5. **批处理唤醒**: 当 DMA 完成指定次数的块传输（Block Transfer）后，产生中断唤醒 CPU。

环境需求
========
该示例的环境需求，可参考 :ref:`环境需求 <general_requirements_cn>`。

硬件连线
==============
本示例配置为 SPI Master 模式，连接 LIS3DH 模块。

* **SPI 接口**:
    * **SCK**: P0_4
    * **MOSI**: P0_2
    * **MISO**: P0_1
    * **CS**: P0_0
* **中断引脚**:
    * **INT1 (Sensor)** -> **P0_6 (MCU Input)**: 传感器数据就绪信号。

.. note::
   SPI 为全双工总线，请确保传感器支持 SPI 模式（LIS3DH 通常通过 CS 引脚状态或配置寄存器选择 SPI/I2C 模式）。

流程图
======
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

配置选项
==============
1. **采样数量配置**:
   
   .. code-block:: c

    /* 每次触发读取 10 组样本 (每组 6 字节) */
    #define TRIGGER_SAMPLE_NUM          (10)

2. **DMA 块计数**:
   
   .. code-block:: c

    /* 触发 10 次 RAP 动作后才产生一次 CPU 中断 */
    #define DMA_BLOCK_COUNTER_NUM       (10)

3. **SPI 引脚定义**:

   .. code-block:: c

    #define SPI_MASTER_SCK_PIN          P0_4
    #define SPI_MASTER_MOSI_PIN         P0_2
    #define SPI_MASTER_MISO_PIN         P0_1
    #define SPI_MASTER_CS_PIN           P0_0

编译和下载
==========
该示例的编译和下载流程，可参考 :ref:`编译和下载 <general_build_download_cn>`。

测试验证
==========================
1. 按照“硬件连线”章节连接好 SPI 设备。
2. 编译并下载固件。
3. 观察串口日志：
   
   * 系统初始化并读取 Sensor ID (0x33)。
   * CPU 进入空闲状态。
   * 当传感器 FIFO 填满并触发足够次数后，DMA 中断发生，打印数据。
   * **注意**: 由于 SPI 是全双工的，第一个接收到的字节对应发送命令时的输入（通常是无效数据或状态），代码在打印时已经自动偏移了该字节。

   ::

     Start Sensor gpio trigger spi transfer sample
     Sensor ID: 0x33
     dma rx handler
     DMA Block Counter Interrupt
     data0: x: 104, y: -50, z: 980
     ...

代码介绍
=======================
源码路径
--------
* 源码路径: :file:`sdk\\sample\\peripheral\\rap\\gpio_trigger_spi_dma\\src`

关键实现细节
------------
1. **SPI Wrap Mode**:
   为了配合 RAP 自动触发，SPI 初始化时必须开启 **Wrap Mode**。这种模式允许 SPI 控制器根据预设的 TX FIFO 内容（通常是读命令）自动进行重复传输，而无需 CPU 每次填充 FIFO。

   .. code-block:: c

    SPI_InitStruct.SPI_WrapModeEn = ENABLE;
    SPI_InitStruct.SPI_TXNDF      = 8; // Number of Data Frames

2. **RAP 级联**:
   * **GPIO -> SPI**: GPIO 输入事件触发 SPI 启动。
   * **SPI -> DMA**: SPI 传输开始事件触发 DMA 开启。这确保了 DMA 仅在 SPI 总线活动时工作。

   .. code-block:: c

    RAP_EventRouteSet(GPIO_IN_EVENT_IN, channel0);
    RAP_ActionBindSet(SPI_MASTER_ACTION_TRANSFER_START, channel0);

    RAP_EventRouteSet(SPI_MASTER_EVENT_TRANSFER_START, channel1);
    RAP_ActionBindSet(SPI_MASTER_RX_DMA_ACTION_START, channel1);

3. **RX 数据处理**:
   SPI 传输包含 1 字节命令头 + N 字节数据。DMA 缓冲区大小设置为 ``(N * Samples) + 1``。在处理数据时，代码特意跳过了每块数据的第 0 个字节（Command Phase 对应的 RX 数据）。

   .. code-block:: c

    /* Indexing Correction for SPI: Skip 1st byte (Dummy) */
    uint32_t data_offset = block_offset + 1 + i * 6;

.. _doxygen-group-list-section:

See Also
==========

相关 API Reference 请查看：

- :ref:`RAP <group___r_a_p>`
- :ref:`SPI <group___s_p_i>`
- :ref:`DMA <group___d_m_a>`
- :ref:`GPIO <group___g_p_i_o>`
- :ref:`RCC <group___r_c_c>`
- :ref:`PINMUX <group___p_i_n_m_u_x>`
- :ref:`NVIC <group___n_v_i_c>`
