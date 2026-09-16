=============================
GPIO In Trigger GPIO Toggle
=============================
该示例演示了如何通过 :term:`RAP` (Real Autonomous Peripheral) 机制，实现 GPIO 输入事件直接触发 GPIO 输出翻转的功能，无需 CPU 干预。

在本示例中，配置 P0_1 为输入引脚，P0_0 为输出引脚。当 P0_1 检测到下降沿信号时，通过 RAP 通道直接触发 P0_0 翻转电平。同时，代码中使用 P0_2 模拟产生脉冲信号输入给 P0_1。

环境需求
========
该示例的环境需求，可参考 :ref:`环境需求 <general_requirements_cn>`。

硬件连线
==============
* 将 P0_2 连接到 P0_1（用于模拟输入信号）。
* 将 P0_0 连接到逻辑分析仪或示波器（用于观察输出效果）。

配置选项
==============
1. 可配置如下宏修改 GPIO 输入和输出引脚。

   .. code-block:: c

    #define OUTPUT_PIN                      P0_0
    #define INPUT_PIN                       P0_1

2. 可配置如下宏修改 RAP 事件和动作映射（需与引脚对应）。

   .. code-block:: c

    #define GPIO_OUT_ACTION_TOGGLE          RAP_ACTION_GPIOA_DRTOGGLE(0)
    #define GPIO_IN_EVENT_IN                RAP_EVENT_GPIOA(1)

编译和下载
==========
该示例的编译和下载流程，可参考 :ref:`编译和下载 <general_build_download_cn>`。


测试验证
==========================
1. 启动开发板，代码会自动在 P0_2 引脚产生脉冲信号。
2. 由于 P0_2 与 P0_1 连接，P0_1 会检测到边沿信号。
3. 通过逻辑分析仪观察 P0_0，可以看到当 P0_1 产生下降沿时，P0_0 的电平发生翻转。
4. （可选）若在代码中将中断配置部分的 ``#if 0`` 改为 ``#if 1``，则每次触发时串口会打印：
   ::
     Enter GPIO_Pin_Handler success

代码介绍
=======================
该章节主要介绍示例中的初始化和相应功能实现的代码和流程说明。

源码路径
--------

工程文件和源码路径如下：

* 工程路径: :file:`sdk\\sample\\peripheral\\rap\\gpio_trigger\\proj`
* 源码路径: :file:`sdk\\sample\\peripheral\\rap\\gpio_trigger\\src`

初始化
------
1. 调用 ``board_gpio_init``，配置 P0_0 (输出)、P0_1 (输入) 和 P0_2 (模拟输出) 的 PAD 和 PINMUX。
2. 调用 ``driver_gpio_init`` 初始化 GPIO 外设：
   
   * 配置 P0_0 为输出模式。
   * 配置 P0_1 为输入模式，开启去抖动功能（Debounce），并设置触发方式为边沿触发（下降沿）。

   .. code-block:: c

    static void driver_gpio_init(void)
    {
        /* Enable GPIO clock */
        RCC_ClockCmd(GPIOA_CLOCK, ENABLE);

        /* Configure GPIO parameters as output mode (P0_0) */
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_StructInit(&GPIO_InitStruct);
        GPIO_InitStruct.GPIO_Pin        = GPIO_OUT_PIN;
        GPIO_InitStruct.GPIO_Dir        = GPIO_DIR_OUT;
        GPIO_InitStruct.GPIO_INTEventEn = DISABLE;
        GPIO_Init(GPIO_OUT_PORT, &GPIO_InitStruct);

        /* Configure GPIO parameters as input mode (P0_1) */
        GPIO_InitStruct.GPIO_Pin        = GPIO_IN_PIN;
        GPIO_InitStruct.GPIO_Dir        = GPIO_DIR_IN;
        GPIO_InitStruct.GPIO_INTEventEn = ENABLE;
        GPIO_InitStruct.GPIO_Trigger    = GPIO_TRIGGER_EDGE;
        GPIO_InitStruct.GPIO_Polarity   = GPIO_POLARITY_ACTIVE_LOW;
        
        /* Configure GPIO Debounce parameters */
        GPIO_InitStruct.GPIO_DebounceEn    = ENABLE;
        /* ... Debounce settings ... */
        
        GPIO_Init(GPIO_IN_PORT, &GPIO_InitStruct);
    }

功能实现
--------
RAP 的配置和触发流程在 ``main`` 函数中实现：

1. 调用 :cpp:any:`RAP_ChannelAllocate` 申请一个 RAP 通道。
2. 调用 :cpp:any:`RAP_EventRouteSet` 将 GPIO 输入事件（P0_1）路由到申请的 RAP 通道。
3. 调用 :cpp:any:`RAP_ActionBindSet` 将 GPIO 输出翻转动作（P0_0）绑定到该 RAP 通道。
4. 调用 :cpp:any:`GPIO_RAPModeCmd` 使能 GPIO 的 RAP 模式。
5. 调用 ``pad_generate_pulse`` 在 P0_2 上产生脉冲，模拟外部输入信号。

   .. code-block:: c

    int main(void)
    {
        /* ... Initialization ... */

        /* Configure RAP channel */
        uint8_t channel0;
        RAP_ChannelAllocate(&channel0);

        /* Route GPIO IN Event to RAP channel */
        RAP_EventRouteSet(GPIO_IN_EVENT_IN, channel0);
        /* Bind GPIO Toggle Action to RAP channel */
        RAP_ActionBindSet(GPIO_OUT_ACTION_TOGGLE, channel0);

        /* Enable GPIO RAP Mode */
        GPIO_RAPModeCmd(GPIO_OUT_PORT, GPIO_OUT_PIN, ENABLE);
        GPIO_RAPModeCmd(GPIO_IN_PORT, GPIO_IN_PIN, ENABLE);

        /* Simulate pulse input using P0_2 */
        pad_generate_pulse(P0_2, 2);

        while (1)
        {
        }
    }

.. _doxygen-group-list-section:

See Also
==========

相关 API Reference 请查看：

- :ref:`RAP <group___r_a_p>`
- :ref:`GPIO <group___g_p_i_o>`
- :ref:`RCC <group___r_c_c>`
- :ref:`PINMUX <group___p_i_n_m_u_x>`
- :ref:`NVIC <group___n_v_i_c>`
