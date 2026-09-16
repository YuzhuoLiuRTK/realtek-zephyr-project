==================================
TIMER Timeout Trigger GPIO Toggle
==================================
该示例演示了如何通过 :term:`RAP` (Real Autonomous Peripheral) 机制，利用 TIMER 的超时（Timeout）事件触发 GPIO 输出翻转，无需 CPU 干预。

此外，本示例还展示了实现 TIMER **单次触发 (One-Shot)** 功能的三种方式：
1. **连续模式（默认）**：TIMER 自动重载，周期性触发 GPIO 翻转。
2. **Action Stop 模式**：通过 RAP 绑定 Stop 动作，在超时同时也触发停止 TIMER。
3. **Shortcut Stop 模式**：通过外设内部 Shortcut 机制，在超时事件发生时直接停止 TIMER。

环境需求
========
该示例的环境需求，可参考 :ref:`环境需求 <general_requirements_cn>`。

硬件连线
==============
* 将 **P0_0** 连接到逻辑分析仪或示波器（用于观察 RAP 触发结果）。

配置选项
==============
1. 可配置如下宏选择 TIMER 的运行模式（连续或单次）。

   .. code-block:: c

    /* 
     * Mode Selection:
     * Both 0: Continuous Mode (Periodic toggling).
     * ...ACTION_STOP = 1: One-Shot using RAP Action.
     * ...SHORTCUT_STOP = 1: One-Shot using Internal Shortcut.
     */
    #define SAMPLE_CONFIG_TIMER_USING_ONE_SHOT_BY_ACTION_STOP     0
    #define SAMPLE_CONFIG_TIMER_USING_ONE_SHOT_BY_SHORTCUT_STOP   0

2. 可配置如下宏修改 TIMER 的触发周期。

   .. code-block:: c

    /* Default is 1 second (40MHz clock / 40,000,000) */
    #define TIMER_PERIOD                    (40000000)

3. 可配置如下宏修改 GPIO 输出引脚。

   .. code-block:: c

    #define OUTPUT_PIN                      P0_0

编译和下载
==========
该示例的编译和下载流程，可参考 :ref:`编译和下载 <general_build_download_cn>`。


测试验证
==========================
1. 启动开发板。
2. 观察 P0_0 的输出波形。

**情况 A：默认配置（连续模式）**
   * 两个 One-Shot 宏均为 0。
   * P0_0 每隔 1 秒翻转一次电平。
   * 输出信号为周期 2 秒、占空比 50% 的方波。

**情况 B：One-Shot 配置（Action 或 Shortcut 模式）**
   * 将其中一个 One-Shot 宏置为 1。
   * P0_0 在 1 秒后电平翻转一次，随后保持不变（TIMER 停止运行）。

代码介绍
=======================
该章节主要介绍示例中的初始化和相应功能实现的代码和流程说明。

源码路径
--------

工程文件和源码路径如下：

* 工程路径: :file:`sdk\\sample\\peripheral\\rap\\timer_timeout_trigger\\proj`
* 源码路径: :file:`sdk\\sample\\peripheral\\rap\\timer_timeout_trigger\\src`

初始化
------
1. 初始化 GPIO (P0_0) 为输出模式。
2. 初始化 TIMER1，配置为用户定义自动重装载模式 (:cpp:any:`TIMER_MODE_USERDEFINE_AUTO`)，周期设为 1 秒。

功能实现
--------
RAP 的配置在 ``main`` 函数中实现：

1. **基础配置**:
   
   * 申请 RAP 通道。
   * 将 TIMER 超时事件 (``TIMER_EVENT_TIMEOUT``) 路由到 RAP 通道。
   * 将 GPIO 翻转动作 (``GPIO_OUT_ACTION_TOGGLE``) 绑定到该通道。

2. **One-Shot 停止机制（可选）**:
   
   * **Shortcut 方式**: 启用 TIMER 内部 Shortcut，将 Event 和 Action 直接短接，无需占用 RAP 动作槽位。
   * **RAP Action 方式**: 将 TIMER 停止动作 (``TIMER_ACTION_STOP``) 也绑定到同一个 RAP 通道。当超时发生时，同时触发 GPIO 翻转和 TIMER 停止。

   .. code-block:: c

    /* Route TIMER Timeout Event to RAP channel */
    RAP_EventRouteSet(TIMER_EVENT_TIMEOUT, channel0);
    /* Bind GPIO Toggle Action to RAP channel */
    RAP_ActionBindSet(GPIO_OUT_ACTION_TOGGLE, channel0);

    /* Configure TIMER One Shot Mode Logic */
    #if (SAMPLE_CONFIG_TIMER_USING_ONE_SHOT_BY_SHORTCUT_STOP == 1)
        /* Method 1: Internal Shortcut */
        TIMER_ShortcutCmd(TIMER_NUM, TIMER_SHORTCUT_ACTION, TIMER_SHORTCUT_EVENT, ENABLE);

    #elif (SAMPLE_CONFIG_TIMER_USING_ONE_SHOT_BY_ACTION_STOP == 1)
        /* Method 2: RAP Action Bind */
        RAP_ActionBindSet(TIMER_ACTION_STOP, channel0);
    #endif

    /* Start TIMER */
    TIMER_ActionTrigger(TIMER_NUM, TIMER_ACTION_START);

.. _doxygen-group-list-section:

See Also
==========

相关 API Reference 请查看：

- :ref:`RAP <group___r_a_p>`
- :ref:`TIMER <group___t_i_m_e_r>`
- :ref:`GPIO <group___g_p_i_o>`
- :ref:`RCC <group___r_c_c>`
- :ref:`PINMUX <group___p_i_n_m_u_x>`
- :ref:`NVIC <group___n_v_i_c>`
