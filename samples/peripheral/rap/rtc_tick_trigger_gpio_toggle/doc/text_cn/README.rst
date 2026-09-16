=============================
RTC Tick Trigger GPIO Toggle
=============================
该示例演示了如何通过 :term:`RAP` (Real Autonomous Peripheral) 机制，利用 RTC 的 Tick 事件（周期性滴答信号）触发 GPIO 输出翻转，无需 CPU 干预。

在本示例中，RTC 被配置为产生周期性的 Tick 信号（默认为 100ms）。每当 RTC Tick 事件发生时，通过 RAP 通道直接触发 P0_0 引脚电平翻转，从而产生方波信号。

环境需求
========
该示例的环境需求，可参考 :ref:`环境需求 <general_requirements_cn>`。

硬件连线
==============
* 将 P0_0 连接到逻辑分析仪或示波器（用于观察输出波形）。

配置选项
==============
1. 可配置如下宏修改 GPIO 输出引脚。

   .. code-block:: c

    #define OUTPUT_PIN                      P0_0

2. 可配置如下宏修改 RTC Tick 的频率。

   .. code-block:: c

    /* 
     * Formula: Tick_Freq = Clock_Src (32kHz) / (PSC + 1)
     * Value (3200 - 1) generates a 10Hz Tick (100ms interval)
     */
    #define RTC_PSC_VALUE                   (3200 - 1)

编译和下载
==========
该示例的编译和下载流程，可参考 :ref:`编译和下载 <general_build_download_cn>`。


测试验证
==========================
1. 启动开发板，RTC 开始运行。
2. 通过逻辑分析仪观察 P0_0。
3. P0_0 将输出占空比为 50% 的方波。
   
   * 电平每 100ms 翻转一次（基于默认的 Tick 配置）。
   * 信号周期为 200ms (5Hz)。

4. （可选）若在代码中将中断配置部分的 ``#if 0`` 改为 ``#if 1``，则每次 Tick 发生时串口会打印：
   ::
     RTC_Handler: RTC_INT_TICK

代码介绍
=======================
该章节主要介绍示例中的初始化和相应功能实现的代码和流程说明。

源码路径
--------

工程文件和源码路径如下：

* 工程路径: :file:`sdk\\sample\\peripheral\\rap\\rtc_tick_trigger\\proj`
* 源码路径: :file:`sdk\\sample\\peripheral\\rap\\rtc_tick_trigger\\src`

初始化
------
1. 调用 ``board_gpio_init`` 和 ``driver_gpio_init`` 初始化 P0_0 为输出模式。
2. 调用 ``driver_rtc_init`` 初始化 RTC：
   
   * 配置预分频器 ``RTC_PSC_VALUE``，设定 Tick 产生的时间间隔。
   * 重置 RTC 计数器。

   .. code-block:: c

    static void driver_rtc_init(void)
    {
        /* Enable RTC clock and DeInit */
        RCC_ClockCmd(RTC_CLOCK, ENABLE);
        RTC_DeInit();

        /* Configure RTC prescaler to generate the tick */
        RTC_SetPrescaler(RTC_PSC_VALUE);

        /* Reset the RTC counter */
        RTC_ResetCounter();
        
        /* ... Optional Interrupt Configuration ... */
    }

功能实现
--------
RAP 的配置和触发流程在 ``main`` 函数中实现：

1. 调用 :cpp:any:`RAP_ChannelAllocate` 申请一个 RAP 通道。
2. 调用 :cpp:any:`RAP_EventRouteSet` 将 RTC Tick 事件 (:c:macro:`RAP_EVENT_RTC_TICK`) 路由到该 RAP 通道。
3. 调用 :cpp:any:`RAP_ActionBindSet` 将 GPIO 输出翻转动作 (``RAP_ACTION_GPIOA_DRTOGGLE(0)``) 绑定到该 RAP 通道。
4. 开启 RTC 和 GPIO 的 RAP 模式。
5. 启动 RTC。

   .. code-block:: c

    int main(void)
    {
        /* ... Initialization ... */

        /* Configure RAP Channel */
        uint8_t channel0;
        RAP_ChannelAllocate(&channel0);

        /* Route RTC Tick Event to RAP channel */
        RAP_EventRouteSet(RTC_EVENT_TICK, channel0);
        /* Bind GPIO Toggle Action to RAP channel */
        RAP_ActionBindSet(GPIO_OUT_ACTION_TOGGLE, channel0);

        /* Enable RAP Mode for RTC and GPIO */
        RTC_RAPModeCmd(ENABLE);
        GPIO_RAPModeCmd(GPIO_OUT_PORT, GPIO_OUT_PIN, ENABLE);

        /* Start RTC */
        RTC_ActionTrigger(RTC_ACTION_START);

        while (1) { }
    }

.. _doxygen-group-list-section:

See Also
==========

相关 API Reference 请查看：

- :ref:`RAP <group___r_a_p>`
- :ref:`RTC <group___r_t_c>`
- :ref:`GPIO <group___g_p_i_o>`
- :ref:`RCC <group___r_c_c>`
- :ref:`PINMUX <group___p_i_n_m_u_x>`
- :ref:`NVIC <group___n_v_i_c>`
