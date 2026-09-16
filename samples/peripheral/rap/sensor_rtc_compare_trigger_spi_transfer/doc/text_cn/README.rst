================================
Sensor RTC Trigger SPI Transfer
================================
该示例演示了如何构建一个完全自动化的**周期性传感器采样系统**。

与使用 GPIO 中断触发不同，本示例利用 :term:`RAP` (Real Autonomous Peripheral) 结合 **RTC (Real-Time Clock)** 定时器，周期性地触发 SPI 接口读取传感器数据，并通过 DMA 将数据搬运至内存。整个过程 CPU 可以处于休眠状态，直到采集满指定数量的数据块。

**工作原理：**
1. **RTC 定时触发**: RTC 被配置为每 1 秒产生一次比较匹配（Compare Match）事件。
2. **RAP 触发 SPI**: RTC 事件通过 RAP 触发 SPI Master 启动传输。SPI 工作在 Wrap Mode，自动发送预设的读取指令。
3. **RAP 触发 DMA**: SPI 的“开始传输”事件级联触发 DMA 通道开启。
4. **数据搬运**: DMA 自动将 SPI 接收到的数据（含 Dummy Byte 和传感器数据）存入 Buffer。
5. **批处理唤醒**: 当采集满 10 次数据（即 10 秒后），DMA 块传输完成中断唤醒 CPU 进行数据处理。

这种机制非常适合环境监测、运动记录等不需要实时处理但需要定时记录的应用场景。

环境需求
========
该示例的环境需求，可参考 :ref:`环境需求 <general_requirements_cn>`。

硬件连线
==============
本示例配置为 SPI Master 模式，连接 LIS3DH 模块。由于是由 RTC 内部触发，**不需要连接传感器的中断引脚**。

* **SPI 接口**:
    * **SCK**: P0_4
    * **MOSI**: P0_2
    * **MISO**: P0_1
    * **CS**: P0_0
* **GND/VCC**: 连接至开发板对应电源引脚。

流程图
======
.. mermaid::

   flowchart TD
      A[RTC Timer] -- 1 Sec Interval --> B(Compare Event)
      B -- RAP Event --> C[SPI Master Start]
      C -- RAP Event --> D[DMA Channel Enable]
      C -- Read Command --> E[LIS3DH Sensor]
      E -- Sensor Data --> C
      D -- Transfer RX Data --> F[RAM Buffer]
      F -- Block Count (10) Done --> G((CPU Interrupt))

配置选项
==============
1. **采样周期配置**:
   通过 RTC 的分频和重装载值设置采样间隔。当前配置为 1 秒。

   .. code-block:: c

    #define RTC_PRESCALER_VALUE         (3200 - 1) /* 100ms Tick */
    #define RTC_COMP_VALUE              10         /* Initial Trigger: 1s */
    #define RTC_COMP_RELOAD_VALUE       10         /* Interval: 1s */

2. **DMA 批处理数量**:
   配置采集多少个样本后唤醒 CPU。

   .. code-block:: c

    #define DMA_BLOCK_COUNTER_NUM       (10)

3. **SPI 采样量**:
   每次触发读取的数据量（1组 X/Y/Z 数据）。

   .. code-block:: c

    #define TRIGGER_SAMPLE_NUM          (1)

编译和下载
==========
该示例的编译和下载流程，可参考 :ref:`编译和下载 <general_build_download_cn>`。

测试验证
==========================
1. 按照“硬件连线”章节连接好 SPI 设备。
2. 编译并下载固件。
3. 观察串口日志：

   * 系统初始化并显示 Sensor ID。
   * **等待约 10 秒**（因为 RTC 每秒触发一次，DMA 需积攒 10 次）。
   * 串口一次性打印出过去 10 秒内的 10 组数据。
     ::
       Start Sensor RTC trigger spi transfer sample
       Sensor ID: 0x33
       dma rx handler
       DMA Block Counter Interrupt
       data0: x: 104, y: -50, z: 980
       data1: x: 102, y: -48, z: 982
       ...

代码介绍
=======================
源码路径
--------
* 源码路径: :file:`sdk\\sample\\peripheral\\rap\\rtc_trigger_spi_dma\\src`

核心逻辑
------------
1. **RTC 配置**:
   RTC 使用 :cpp:any:`RTC_EnableCompAutoReload` 配置为自动重装载模式，确保时间基准的连续性，无需软件干预重置计数器。

2. **RAP 级联**:
   建立了 **RTC -> SPI -> DMA** 的触发链。
   
   .. code-block:: c

    /* Channel 0: RTC Compare -> SPI Start */
    RAP_EventRouteSet(RTC_EVENT_COMPARE, channel0);
    RAP_ActionBindSet(SPI_MASTER_ACTION_TRANSFER_START, channel0);

    /* Channel 1: SPI Start -> DMA Enable */
    RAP_EventRouteSet(SPI_MASTER_EVENT_TRANSFER_START, channel1);
    RAP_ActionBindSet(SPI_MASTER_RX_DMA_ACTION_START, channel1);

3. **数据偏移处理**:
   由于 SPI 是全双工的，每次读取操作都需要发送 1 字节命令。这导致接收缓冲区中每组数据的第一个字节是无效的（Dummy Byte）。在打印日志时，代码通过偏移量跳过该字节。

   .. code-block:: c

    /* Block Size = 1 (Dummy) + 6 (Data) */
    uint32_t block_offset = j * (1 + BYTES_PER_SAMPLE * TRIGGER_SAMPLE_NUM);
    /* Real Data starts at block_offset + 1 */

.. _doxygen-group-list-section:

See Also
==========

相关 API Reference 请查看：

- :ref:`RAP <group___r_a_p>`
- :ref:`RTC <group___r_t_c>`
- :ref:`SPI <group___s_p_i>`
- :ref:`DMA <group___d_m_a>`
- :ref:`RCC <group___r_c_c>`
- :ref:`PINMUX <group___p_i_n_m_u_x>`
- :ref:`NVIC <group___n_v_i_c>`
