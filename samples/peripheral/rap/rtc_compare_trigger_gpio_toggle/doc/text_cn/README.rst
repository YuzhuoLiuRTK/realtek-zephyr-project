================================
RTC Compare Trigger GPIO Toggle
================================
该示例演示了如何通过 :term:`RAP` (Real Autonomous Peripheral) 机制，利用 RTC 的比较匹配事件（Compare Match）触发 GPIO 输出翻转，无需 CPU 干预。

在本示例中，配置 RTC 每隔一定时间产生比较匹配事件，通过 RAP 通道直接触发 P0_0 引脚电平翻转。同时演示了三种 RTC 比较值的重载（Reload）机制。

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

2. 可配置如下宏选择 RTC 比较值重载模式（三选一）。

   .. code-block:: c

    #define SAMPLE_CONFIG_RTC_USING_RELOAD_BY_AUTO_RELOAD         1
    #define SAMPLE_CONFIG_RTC_USING_RELOAD_BY_ACTION_RELOAD       0
    #define SAMPLE_CONFIG_RTC_USING_RELOAD_BY_SHORTCUT_RELOAD     0

   * **AUTO_RELOAD**: RTC 硬件自动在比较匹配时重载预设值。
   * **ACTION_RELOAD**: 比较事件通过 RAP 触发特定的重载动作（Action）来执行重载。
   * **SHORTCUT_RELOAD**: 比较事件通过内部捷径（Shortcut）直接触发重载任务。

3. 可配置如下宏修改 RTC 定时参数。

   .. code-block:: c

    #define RTC_PSC_VALUE                   (3200 - 1)  /* 100ms per tick */
    #define RTC_COMP_VALUE                  (20)        /* Timeout: 2.0s */
    #define RTC_COMP_RELOAD_VALUE           (10)        /* Reload: 1.0s */

编译和下载
==========
该示例的编译和下载流程，可参考 :ref:`编译和下载 <general_build_download_cn>`。


测试验证
==========================
1. 启动开发板，RTC 开始运行。
2. 通过逻辑分析仪观察 P0_0：
   
   * 初始阶段，RTC 计数达到 2.0s (COMP_VALUE) 时，触发 P0_0 翻转。
   * 随后 RTC 重载 1.0s (RELOAD_VALUE)，之后每隔 1.0s 触发一次 P0_0 翻转。

3. （可选）若在代码中启用中断，串口将打印 RTC 比较中断信息。

代码介绍
=======================
该章节主要介绍示例中的初始化和相应功能实现的代码和流程说明。

源码路径
--------

工程文件和源码路径如下：

* 工程路径: :file:`sdk\\sample\\peripheral\\rap\\rtc_trigger\\proj`
* 源码路径: :file:`sdk\\sample\\peripheral\\rap\\rtc_trigger\\src`

初始化
------
1. 调用 ``board_gpio_init`` 和 ``driver_gpio_init`` 初始化 P0_0 为输出模式。
2. 调用 ``driver_rtc_init`` 初始化 RTC：
   
   * 配置预分频器 ``RTC_PSC_VALUE``，设定 Tick 频率。
   * 配置比较值 ``RTC_COMP_VALUE`` 和重载值 ``RTC_COMP_RELOAD_VALUE``。
   * 重置 RTC 计数器。

   .. code-block:: c

    static void driver_rtc_init(void)
    {
        /* Enable RTC clock and DeInit */
        RCC_ClockCmd(RTC_CLOCK, ENABLE);
        RTC_DeInit();

        /* Configure RTC prescaler */
        RTC_SetPrescaler(RTC_PSC_VALUE);

        /* Configure RTC compare and reload value */
        RTC_SetCompValue(RTC_COMP_NUM, RTC_COMP_VALUE);
        RTC_SetCompReloadValue(RTC_COMP_NUM, RTC_COMP_RELOAD_VALUE);

        /* Reset the RTC counter */
        RTC_ResetCounter();
        
        /* ... Optional Interrupt Configuration ... */
    }

功能实现
--------
RAP 的配置和触发流程在 ``main`` 函数中实现：

1. 调用 :cpp:any:`RAP_ChannelAllocate` 申请一个 RAP 通道。
2. 调用 :cpp:any:`RAP_EventRouteSet` 将 RTC 比较事件路由到该 RAP 通道。
3. 调用 :cpp:any:`RAP_ActionBindSet` 将 GPIO 输出翻转动作绑定到该 RAP 通道。
4. 根据宏定义配置 RTC 重载机制（Auto/Action/Shortcut）。
5. 开启 RTC 和 GPIO 的 RAP 模式，并启动 RTC。

   .. code-block:: c

    int main(void)
    {
        /* ... Initialization ... */

        /* Route RTC Compare event and Bind GPIO Toggle action */
        RAP_EventRouteSet(RTC_EVENT_COMPARE, channel0);
        RAP_ActionBindSet(GPIO_OUT_ACTION_TOGGLE, channel0);

        /* Configure RTC Comparator Reload Mechanism */
    #if (SAMPLE_CONFIG_RTC_USING_RELOAD_BY_AUTO_RELOAD == 1)
        RTC_CompAutoReloadCmd(RTC_COMP_NUM, ENABLE);
    #elif (SAMPLE_CONFIG_RTC_USING_RELOAD_BY_ACTION_RELOAD == 1)
        RAP_ActionBindSet(RTC_ACTION_RELOAD, channel0);
    #elif (SAMPLE_CONFIG_RTC_USING_RELOAD_BY_SHORTCUT_RELOAD == 1)
        RTC_ShortcutCmd(RTC_SHORTCUT_ACTION_RELOAD, RTC_SHORTCUT_EVENT_COMPARE, ENABLE);
    #endif

        /* Enable RAP Mode */
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
