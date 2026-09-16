=========================================
TIMER Timeout Trigger ADC Oneshot Sample
=========================================
该示例演示了如何通过 :term:`RAP` (Real Autonomous Peripheral) 机制实现外设间的级联触发，无需 CPU 参与。

具体流程如下：
1. **TIMER 触发**: TIMER1 周期性（每 1 秒）产生超时事件。
2. **ADC 采样**: TIMER 超时事件通过 RAP 通道 0 触发 ADC 进行一次单次采样（One-Shot Sample）。
3. **GPIO 翻转**: 当 ADC 采样完成（Done）事件发生时，通过 RAP 通道 1 触发 GPIO 输出翻转。

这种机制非常适合需要精确定时采样且希望减少 CPU 负载的应用场景。

环境需求
========
该示例的环境需求，可参考 :ref:`环境需求 <general_requirements_cn>`。

硬件连线
==============
* **模拟输入**: 将 **P2_0** 连接到待测电压源（如 1.5V 或电位器中间脚）。
* **观察引脚**: 将 **P0_0** 连接到逻辑分析仪或示波器（用于观察 RAP 级联触发结果）。

配置选项
==============
1. 可配置如下宏修改 TIMER 的触发周期。

   .. code-block:: c

    /* Default is 1 second (40MHz clock / 40,000,000) */
    #define TIMER_PERIOD                    (40000000)

2. 可配置如下宏修改 ADC 输入引脚和通道。

   .. code-block:: c

    #define ADC_PIN                         P2_0
    #define ADC_CHANNEL                     ADC_Channel_Index_0

3. 可配置如下宏修改 GPIO 输出引脚。

   .. code-block:: c

    #define OUTPUT_PIN                      P0_0

编译和下载
==========
该示例的编译和下载流程，可参考 :ref:`编译和下载 <general_build_download_cn>`。


测试验证
==========================
1. 按照“硬件连线”章节连接好设备。
2. 启动开发板。
3. 观察 P0_0 的输出波形：
   
   * P0_0 每隔 1 秒翻转一次电平（由 TIMER -> ADC -> GPIO 链路触发）。
   * 输出信号为周期 2 秒、占空比 50% 的方波。

4. （可选）若开启 ADC 中断 (``#if 1``)，串口会打印采样数据：
   ::
     ADC_Handler
     ADC_INT_ONE_SHOT_DONE -> sample_data: ...

代码介绍
=======================
该章节主要介绍示例中的初始化和相应功能实现的代码和流程说明。

源码路径
--------

工程文件和源码路径如下：

* 工程路径: :file:`sdk\\sample\\peripheral\\rap\\timer_trigger_adc\\proj`
* 源码路径: :file:`sdk\\sample\\peripheral\\rap\\timer_trigger_adc\\src`

初始化
------
1. 初始化 GPIO (P0_0) 为输出模式。
2. 初始化 TIMER1，配置为用户定义自动重装载模式 (:cpp:any:`TIMER_MODE_USERDEFINE_AUTO`)，周期设为 1 秒。
3. 初始化 ADC (P2_0)，配置为单次采样模式，使能 Schedule Index 0。

功能实现
--------
RAP 的级联配置在 ``main`` 函数中实现，使用了两个 RAP 通道：

1. **通道 0 (Timer -> ADC)**:
   
   * 将 TIMER 超时事件 (``TIMER_EVENT_TIMEOUT``) 路由到通道 0。
   * 将 ADC 采样动作 (``ADC_ACTION_SAMPLE``) 绑定到通道 0。

2. **通道 1 (ADC -> GPIO)**:
   
   * 将 ADC 完成事件 (``ADC_EVENT_DONE``) 路由到通道 1。
   * 将 GPIO 翻转动作 (``GPIO_OUT_ACTION_TOGGLE``) 绑定到通道 1。

3. **启动**:
   * 开启各外设的 RAP 模式。
   * 启动 TIMER，开始周期性触发流程。

   .. code-block:: c

    /* ... Initialization ... */
    
    /* Route TIMER Timeout Event to RAP channel0 to trigger ADC */
    RAP_EventRouteSet(TIMER_EVENT_TIMEOUT, channel0);
    RAP_ActionBindSet(ADC_ACTION_SAMPLE, channel0);

    /* Route ADC Done Event to RAP channel1 to trigger GPIO */
    RAP_EventRouteSet(ADC_EVENT_DONE, channel1);
    RAP_ActionBindSet(GPIO_OUT_ACTION_TOGGLE, channel1);

    /* Enable RAP Mode */
    TIMER_RAPModeCmd(TIMER_NUM, ENABLE);
    ADC_RAPModeCmd(ADC, ENABLE);
    GPIO_RAPModeCmd(GPIO_OUT_PORT, GPIO_OUT_PIN, ENABLE);

    /* Start TIMER */
    TIMER_ActionTrigger(TIMER_NUM, TIMER_ACTION_START);

.. _doxygen-group-list-section:

See Also
==========

相关 API Reference 请查看：

- :ref:`RAP <group___r_a_p>`
- :ref:`TIMER <group___t_i_m_e_r>`
- :ref:`ADC <group___a_d_c>`
- :ref:`GPIO <group___g_p_i_o>`
- :ref:`RCC <group___r_c_c>`
- :ref:`PINMUX <group___p_i_n_m_u_x>`
- :ref:`NVIC <group___n_v_i_c>`
