===============================
TIMER Latch Trigger GPIO Toggle
===============================
该示例演示了如何通过 :term:`RAP` (Real Autonomous Peripheral) 机制，利用 TIMER 的锁存（Latch）FIFO 阈值事件触发 GPIO 输出翻转，无需 CPU 干预。

在本示例中，TIMER 被配置为锁存模式（Latch Mode），用于捕获输入信号的脉冲。当捕获的脉冲数量达到预设阈值（默认 4 个）时，产生阈值事件，通过 RAP 通道直接触发 P0_0 引脚电平翻转。同时演示了如何通过 ISR 或 DMA 读取锁存的数据。

环境需求
========
该示例的环境需求，可参考 :ref:`环境需求 <general_requirements_cn>`。

硬件连线
==============
本示例包含信号发生（模拟）和信号捕获两部分，需要进行如下硬件连接：

* **信号回路**: 将 **P0_2** (模拟脉冲输出) 连接到 **P0_1** (TIMER 锁存输入)。
* **观察引脚**: 将 **P0_0** 连接到逻辑分析仪或示波器（用于观察 RAP 触发结果）。

配置选项
==============
1. 可配置如下宏选择锁存数据的读取模式（二选一）。

   .. code-block:: c

    #define SAMPLE_CONFIG_TIMER_LATCH_MODE_ISR       1
    #define SAMPLE_CONFIG_TIMER_LATCH_MODE_DMA       0

   * **ISR Mode**: 阈值事件触发中断，CPU 在中断服务函数中读取 FIFO 数据。
   * **DMA Mode**: 阈值事件触发 DMA 请求，DMA 自动将 FIFO 数据搬运至内存。

2. 可配置如下宏修改触发阈值和引脚。

   .. code-block:: c

    #define TIMER_LATCH_TRIGGER_THRESHOLD   4       /* 触发 RAP 事件所需的脉冲数 */
    #define TIMER_LATCH_TRIGGER_PAD         P0_1    /* 捕获输入引脚 */

编译和下载
==========
该示例的编译和下载流程，可参考 :ref:`编译和下载 <general_build_download_cn>`。


测试验证
==========================
1. 按照“硬件连线”章节连接 P0_2 和 P0_1。
2. 启动开发板。
3. 代码运行后，P0_2 会自动产生 4 个脉冲信号。
4. 观察 P0_0 和串口日志：
   
   * **波形**: 当 TIMER 在 P0_1 检测到第 4 个脉冲的上升沿时，P0_0 电平发生翻转。
   * **日志**: 串口打印读取到的锁存数据（ISR 或 DMA 模式）。
     ::
       TIMER_Handler
       timer_latch_data[0] = 0x...
       ...

代码介绍
=======================
该章节主要介绍示例中的初始化和相应功能实现的代码和流程说明。

源码路径
--------

工程文件和源码路径如下：

* 工程路径: :file:`sdk\\sample\\peripheral\\rap\\timer_latch_trigger\\proj`
* 源码路径: :file:`sdk\\sample\\peripheral\\rap\\timer_latch_trigger\\src`

初始化
------
1. 调用 ``driver_timer_init`` 初始化 TIMER：
   
   * 配置为 :cpp:any:`TIMER_MODE_FREERUN`。
   * 开启锁存功能 (Latch)，设置触发边沿、阈值 (``TIMER_LATCH_TRIGGER_THRESHOLD``) 和输入引脚。
   * 根据配置选择开启 DMA 请求或 NVIC 中断。

   .. code-block:: c

    static void driver_timer_init(void)
    {
        /* ... Basic Config ... */
        TIMER_InitStruct.TIMER_Latch.TIMER_LatchEn[0] = ENABLE;
        TIMER_InitStruct.TIMER_Latch.TIMER_LatchTriggerMode[0] = TIMER_LATCH_TRIGGER_RISING_EDGE;
        TIMER_InitStruct.TIMER_Latch.TIMER_LatchThreshold = TIMER_LATCH_TRIGGER_THRESHOLD;
        TIMER_InitStruct.TIMER_Latch.TIMER_LatchTriggerPad = TIMER_LATCH_TRIGGER_PAD;
        
        /* ... DMA or NVIC Config ... */
        TIMER_TimeBaseInit(TIMER_NUM, &TIMER_InitStruct);
    }

功能实现
--------
RAP 的配置和脉冲模拟在 ``main`` 函数中实现：

1. **RAP 配置**:
   
   * 申请 RAP 通道。
   * 将 TIMER 锁存阈值事件 (``TIMER_EVENT_LATCH_THRESHOLD``) 路由到 RAP 通道。
   * 将 GPIO 翻转动作绑定到该通道。

   .. code-block:: c

    RAP_EventRouteSet(TIMER_EVENT_LATCH_THRESHOLD, channel0);
    RAP_ActionBindSet(GPIO_OUT_ACTION_TOGGLE, channel0);
    TIMER_RAPModeCmd(TIMER_NUM, ENABLE);
    GPIO_RAPModeCmd(GPIO_OUT_PORT, GPIO_OUT_PIN, ENABLE);

2. **触发源模拟**:
   
   * 启动 TIMER 后，调用 ``pad_generate_pulse`` 函数在 P0_2 上产生指定数量的脉冲，模拟外部输入信号。

   .. code-block:: c

    /* Start TIMER */
    TIMER_ActionTrigger(TIMER_NUM, TIMER_ACTION_START);

    /* Simulate pulse input */
    pad_generate_pulse(P0_2, TIMER_LATCH_TRIGGER_THRESHOLD);

.. _doxygen-group-list-section:

See Also
==========

相关 API Reference 请查看：

- :ref:`RAP <group___r_a_p>`
- :ref:`TIMER <group___t_i_m_e_r>`
- :ref:`DMA <group___d_m_a>`
- :ref:`GPIO <group___g_p_i_o>`
- :ref:`RCC <group___r_c_c>`
- :ref:`PINMUX <group___p_i_n_m_u_x>`
- :ref:`NVIC <group___n_v_i_c>`
