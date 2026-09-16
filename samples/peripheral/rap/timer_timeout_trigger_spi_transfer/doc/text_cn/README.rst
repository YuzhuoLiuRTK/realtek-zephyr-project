===================================
TIMER Timeout Trigger SPI Transfer
===================================
该示例演示了如何通过 :term:`RAP` (Real Autonomous Peripheral) 机制实现外设间的联动和级联触发。

具体流程如下：
1. **TIMER 触发 SPI**: 利用 TIMER 的超时事件作为源，通过 RAP 触发 SPI Master 启动一次数据传输 (Wrap Mode)。
2. **SPI 触发 GPIO**: 利用 SPI 传输过程中的“开始”和“结束”事件，通过 RAP 触发 GPIO 引脚翻转。

这种机制展示了 RAP 强大的事件路由能力：
*   **级联**: Timer -> SPI -> GPIO。
*   **多对一映射**: SPI 的 Start 事件和 End 事件被路由到同一个 RAP 通道，触发同一个 GPIO 翻转动作。这使得 GPIO 输出的波形可以精确包络 SPI 的传输帧。

环境需求
========
该示例的环境需求，可参考 :ref:`环境需求 <general_requirements_cn>`。

硬件连线
==============
本示例配置为 SPI Master 模式，建议连接逻辑分析仪观察时序，或者将 MISO 与 MOSI 短接进行回环测试。

* **SPI 接口**:
    * **SCK**: P0_1
    * **MOSI**: P0_2
    * **MISO**: P0_4 (若需测试接收数据，可连接至 MOSI)
    * **CS**: P0_5
* **GPIO 指示**:
    * **Output**: P0_0 (用于观察 RAP 触发的翻转信号)
* **GND**: 确保共地。

配置选项
==============
1. 可配置如下宏修改 TIMER 的触发周期。

   .. code-block:: c

    #define TIMER_PERIOD                (40000000) /* 1 Second */

2. 可配置 SPI Wrap Mode 的数据帧长度。

   .. code-block:: c

    #define SPI_WRAP_NDF_LEN            8   /* 传输 8 个数据帧 */

3. 可配置 GPIO 输出引脚。

   .. code-block:: c

    #define OUTPUT_PIN                  P0_0

编译和下载
==========
该示例的编译和下载流程，可参考 :ref:`编译和下载 <general_build_download_cn>`。


测试验证
==========================
1. 按照“硬件连线”章节连接好引脚，推荐使用逻辑分析仪同时抓取 SPI 引脚和 P0_0。
2. 启动开发板。
3. 观察现象：
   
   * **时序**: 每隔 1 秒，SPI 总线产生一次数据传输。
   * **GPIO (P0_0)**: 
     * 在 SPI 传输开始时 (CS 拉低前后) 翻转一次。
     * 在 SPI 传输结束时 (CS 拉高前后) 再次翻转。
     * 效果上，P0_0 的电平变化“框”住了 SPI 的传输过程。
   * **日志**: 如果进行了回环连接 (MOSI-MISO)，串口将打印接收到的数据。
     ::
       Start RAP timer timeout trigger SPI transfer sample
       SPI_MASTER_Handler
       SPI Master RX Length: 8, Data[0]: ...

代码介绍
=======================
该章节主要介绍示例中的初始化和相应功能实现的代码和流程说明。

源码路径
--------

工程文件和源码路径如下：

* 工程路径: :file:`sdk\\sample\\peripheral\\rap\\timer_trigger_spi\\proj`
* 源码路径: :file:`sdk\\sample\\peripheral\\rap\\timer_trigger_spi\\src`

初始化
------
1. **SPI Master 初始化**: 
   * 配置为 Full Duplex Master 模式。
   * 开启 **Wrap Mode** (``SPI_WrapModeEn = ENABLE``)，这是配合 RAP 自动传输的关键模式。
   * 预填充 TX FIFO，并设置 Wrap Mode 的传输参数。

   .. code-block:: c

    /* Configure SPI RAP Action Transfer parameters */
    /* Params: SPIx, CmdLength, WaitCount, TransferLength */
    SPI_SetActionTransfer(SPI_MASTER, 1, 10, 7);
    
    /* Pre-fill TX FIFO & Set NDF */
    SPI_SendBuffer(SPI_MASTER, spi_master_tx_buffer, 1);
    SPI_WrapModeSetTxNdf(SPI_MASTER, SPI_WRAP_NDF_LEN);

2. **GPIO & Timer 初始化**: 常规配置，确保时钟开启。

功能实现
--------
RAP 的配置在 ``main`` 函数中实现，使用了两个 RAP 通道来建立级联关系：

1. **通道 0 (Timer -> SPI)**:
   * **源事件**: TIMER 超时 (``TIMER_EVENT_TIMEOUT``)。
   * **动作**: SPI 启动传输 (:cpp:any:`SPI_ACTION_START`)。

2. **通道 1 (SPI -> GPIO)**:
   * **源事件**: SPI 传输开始 (:cpp:any:`SPI_EVENT_START`) **或** SPI 传输结束 (:cpp:any:`SPI_EVENT_END`)。
   * **动作**: GPIO 引脚电平翻转 (``GPIO_ACTION_TOGGLE``)。
   * **注意**: 这里展示了 RAP 的“或”逻辑，即多个事件路由到同一通道，触发相同动作。

   .. code-block:: c

    /* Channel 0: Timer Timeout triggers SPI Start */
    RAP_EventRouteSet(TIMER_EVENT_TIMEOUT, channel0);
    RAP_ActionBindSet(SPI_ACTION_START, channel0);

    /* Channel 1: SPI Start OR SPI End triggers GPIO Toggle */
    RAP_EventRouteSet(SPI_EVENT_START, channel1);
    RAP_EventRouteSet(SPI_EVENT_END, channel1);
    RAP_ActionBindSet(GPIO_ACTION_TOGGLE, channel1);

    /* Enable RAP Mode */
    TIMER_RAPModeCmd(TIMER_NUM, ENABLE);
    SPI_RAPModeCmd(SPI_MASTER, ENABLE);
    GPIO_RAPModeCmd(GPIOA, GPIO_OUT_PIN, ENABLE);

.. _doxygen-group-list-section:

See Also
==========

相关 API Reference 请查看：

- :ref:`RAP <group___r_a_p>`
- :ref:`TIMER <group___t_i_m_e_r>`
- :ref:`SPI <group___s_p_i>`
- :ref:`GPIO <group___g_p_i_o>`
- :ref:`RCC <group___r_c_c>`
- :ref:`PINMUX <group___p_i_n_m_u_x>`
- :ref:`NVIC <group___n_v_i_c>`
