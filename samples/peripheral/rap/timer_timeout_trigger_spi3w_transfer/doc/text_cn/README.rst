====================================
TIMER Timeout Trigger SPI3W Transfer
====================================
该示例演示了如何通过 :term:`RAP` (Real Autonomous Peripheral) 机制，利用 TIMER 定时触发 SPI3W（3-Wire SPI）接口的快速突发读取（Quick Burst Read）操作，并利用 SPI3W 的传输结束事件级联触发 GPIO 翻转。

具体流程如下：
1. **TIMER 触发 SPI3W**: TIMER 每隔 1 秒产生一次超时事件，通过 RAP 触发 SPI3W 启动一次 Quick Burst 读取序列。
2. **SPI3W 触发 GPIO**: 当 SPI3W 读取完成后，产生结束事件，通过 RAP 触发 GPIO 引脚翻转，作为传输完成的指示信号。

这种机制非常适合需要周期性读取传感器数据的低功耗应用场景，无需 CPU 频繁参与启动过程。

环境需求
========
该示例的环境需求，可参考 :ref:`环境需求 <general_requirements_cn>`。

硬件连线
==============
本示例使用 SPI3W 接口（通常用于连接光学传感器等设备），建议连接逻辑分析仪观察时序。

* **SPI3W 接口**:
    * **CLK**: P0_1
    * **DATA**: P0_2
    * **QB (Quick Burst Trigger)**: P0_4 (该引脚在本例中用于输出 Burst 脉冲信号)
* **GPIO 指示**:
    * **Output**: P0_0 (用于观察 RAP 触发的翻转信号)
* **GND**: 确保共地。

配置选项
==============
1. 可配置如下宏修改 TIMER 的触发周期。

   .. code-block:: c

    #define TIMER_PERIOD                (40000000) /* 1 Second @ 40MHz */

2. 可配置 SPI3W 的通信速率和读取延迟。

   .. code-block:: c

    #define SPI3W_SPEED                 800000    /* 800kHz */
    #define SPI3W_READ_DELAY            3         /* 2.5us Delay */

3. 可配置 Quick Burst 读取的数据长度。

   .. code-block:: c

    /* Read 3 bytes */
    SPI3W_SetQuickBurstRead(3, ENABLE);

编译和下载
==========
该示例的编译和下载流程，可参考 :ref:`编译和下载 <general_build_download_cn>`。


测试验证
==========================
1. 按照“硬件连线”章节连接好引脚，推荐使用逻辑分析仪。
2. 启动开发板。
3. 观察现象：
   
   * **时序**: 每隔 1 秒，P0_4 (QB) 引脚输出一个约 5us 的脉冲，随后 SPI3W 开始时钟和数据传输。
   * **GPIO (P0_0)**: 在每次 SPI3W 传输结束后翻转一次状态。
   * **日志**: 如果连接了 SPI3W 从设备或进行了仿真，串口将打印读取到的数据。
     ::
       Start RAP timer timeout trigger SPI3W transfer sample
       SPI3W_Handler
       SPI3W RX Length 3, Data[0]: ...

代码介绍
=======================
该章节主要介绍示例中的初始化和相应功能实现的代码和流程说明。

源码路径
--------

工程文件和源码路径如下：

* 工程路径: :file:`sdk\\sample\\peripheral\\rap\\timer_trigger_spi3w\\proj`
* 源码路径: :file:`sdk\\sample\\peripheral\\rap\\timer_trigger_spi3w\\src`

初始化
------
1. **SPI3W 初始化**: 
   * 配置为 2-Wire 模式 (Clock + Data)，但使用额外引脚作为 Quick Burst 触发信号。
   * 设置通信速率为 800kHz。
   * 配置 **Quick Burst Read** 参数：设置读取长度为 3 字节，脉冲宽度为 19 个周期 (约 5us)。

   .. code-block:: c

    /* Configure Quick Burst Read */
    SPI3W_SetQuickBurstRead(3, ENABLE);
    SPI3W_SetQuickBurstPulseWidth(19);

2. **GPIO & Timer 初始化**: 配置 P0_0 为输出，Timer 为 1 秒周期自动重装载模式。

功能实现
--------
RAP 的配置在 ``main`` 函数中实现，使用了两个 RAP 通道：

1. **通道 0 (Timer -> SPI3W)**:
   * **源事件**: TIMER 超时 (``TIMER_EVENT_TIMEOUT``)。
   * **动作**: SPI3W 启动 (:cpp:any:`SPI3W_ACTION_START`)，这将触发 Quick Burst 序列。

2. **通道 1 (SPI3W -> GPIO)**:
   * **源事件**: SPI3W 传输结束 (``SPI3W_EVENT_END``)。
   * **动作**: GPIO 引脚电平翻转 (``GPIO_ACTION_TOGGLE``)。

   .. code-block:: c

    /* Channel 0: Timer Timeout triggers SPI3W Start */
    RAP_EventRouteSet(TIMER_EVENT_TIMEOUT, channel0);
    RAP_ActionBindSet(SPI3W_ACTION_START, channel0);

    /* Channel 1: SPI3W End triggers GPIO Toggle */
    RAP_EventRouteSet(SPI3W_EVENT_END, channel1);
    RAP_ActionBindSet(GPIO_ACTION_TOGGLE, channel1);

    /* Enable RAP Mode */
    TIMER_RAPModeCmd(TIMER_NUM, ENABLE);
    SPI3W_RAPModeCmd(ENABLE);
    GPIO_RAPModeCmd(GPIOA, GPIO_PIN_BIT, ENABLE);

.. _doxygen-group-list-section:

See Also
==========

相关 API Reference 请查看：

- :ref:`RAP <group___r_a_p>`
- :ref:`TIMER <group___t_i_m_e_r>`
- :ref:`SPI3W <group___s_p_i3_w>`
- :ref:`GPIO <group___g_p_i_o>`
- :ref:`RCC <group___r_c_c>`
- :ref:`PINMUX <group___p_i_n_m_u_x>`
- :ref:`NVIC <group___n_v_i_c>`
