==================================
TIMER Timeout Trigger IR TX Start
==================================
该示例演示了如何通过 :term:`RAP` (Real Autonomous Peripheral) 机制，利用 TIMER 的超时事件触发红外 (IR) 模块的发送动作 (TX)，配合 DMA 自动搬运预先生成的 NEC 协议波形数据，实现无需 CPU 介入的周期性红外信号发射。

在本示例中，CPU 预先计算并填充好 NEC 协议的波形数据到内存 Buffer 中。TIMER 每隔 1 秒产生一次触发信号，通过 RAP 启动 IR 发送。IR 模块在发送过程中通过 DMA 请求自动获取后续波形数据。

主要流程：
1. **波形生成**: CPU 计算 NEC 协议波形数据 (Leader, Data, Stop) 并存入内存 Buffer。
2. **预配置**: 配置 IR 为 TX 模式并启用 DMA 请求；配置 DMA 将内存数据搬运至 IR TX FIFO。
3. **触发**: TIMER 每隔 1 秒产生超时事件，通过 RAP 触发 IR 开始发送。
4. **传输**: IR 模块消耗 FIFO 数据并输出波形，同时触发 DMA 补充数据。
5. **完成**: 当所有数据发送完毕，DMA 产生传输完成中断。

环境需求
========
该示例的环境需求，可参考 :ref:`环境需求 <general_requirements_cn>`。

硬件连线
==============
本示例为红外发射演示，需要连接示波器或红外接收器进行观察：

* **信号输出**: 将 **P0_0** (IR TX) 连接到示波器探头或红外接收模块的数据引脚。
* **GND**: 确保共地。

配置选项
==============
1. 可配置如下宏修改红外载波频率。

   .. code-block:: c

    /* IR Carrier Frequency: 38kHz */
    IR_InitStruct.IR_Freq = 38000;

2. 可配置如下宏修改 TIMER 的触发周期。

   .. code-block:: c

    #define TIMER_PERIOD                40000000 /* 1 Second @ 40MHz */

3. 可在 ``main`` 函数中修改发送的 NEC 数据 Payload。

   .. code-block:: c

    uint32_t nec_payload = 0x00F720DF;

编译和下载
==========
该示例的编译和下载流程，可参考 :ref:`编译和下载 <general_build_download_cn>`。


测试验证
==========================
1. 按照“硬件连线”章节连接好设备。
2. 启动开发板。
3. 观察现象：
   
   * **示波器**: 每隔 1 秒可以看到 P0_0 引脚输出一串 38kHz 载波调制的 NEC 协议波形。
   * **串口日志**: 每次发送完成（DMA 传输结束）会打印日志。
     ::
       Start RAP timer timeout trigger ir tx sample
       IR_TX_DMA_Handler: IR TX Done
       IR_TX_DMA_Handler: IR TX Done
       ...

代码介绍
=======================
该章节主要介绍示例中的初始化和相应功能实现的代码和流程说明。

源码路径
--------

工程文件和源码路径如下：

* 工程路径: :file:`sdk\\sample\\peripheral\\rap\\timer_trigger_ir_tx\\proj`
* 源码路径: :file:`sdk\\sample\\peripheral\\rap\\timer_trigger_ir_tx\\src`

初始化
------
1. **IR 初始化**: 配置为 TX 模式，设置载波频率为 38kHz，启用 DMA 功能。
2. **DMA 初始化**: 配置 DMA 通道从内存 Buffer (``ir_tx_data_buffer``) 搬运数据到 ``IR->IR_TX_FIFO``。
3. **波形生成**: ``ir_generate_nec_waveform`` 函数负责将 32 位 NEC 数据转换为 IR 硬件所需的波形编码格式（Mark/Space时长）。

   .. code-block:: c

    /* Generate NEC Waveform */
    uint32_t ir_tx_data_length = ir_generate_nec_waveform(nec_payload, ir_tx_data_buffer);

功能实现
--------
RAP 的配置在 ``main`` 函数中实现：

1. **RAP 路由**:
   
   * 将 TIMER 超时事件 (``TIMER_EVENT_TIMEOUT``) 路由到 RAP 通道。
   * 将 IR 启动发送动作 (:c:macro:`RAP_ACTION_IR_START_TX`) 绑定到该通道。

2. **启动**:
   * 开启 TIMER 和 IR 的 RAP 模式。
   * 启动 DMA 通道准备传输。
   * 启动 TIMER，之后 IR 发送将由硬件定时触发。

   .. code-block:: c

    /* Route TIMER Timeout Event to RAP channel */
    RAP_EventRouteSet(TIMER_EVENT_TIMEOUT, channel0);
    /* Bind IR START TX Action to RAP channel */
    RAP_ActionBindSet(RAP_ACTION_IR_START_TX, channel0);

    /* Enable RAP Mode */
    TIMER_RAPModeCmd(TIMER_NUM, ENABLE);
    IR_RAPModeCmd(ENABLE);

    /* Start TIMER loop */
    TIMER_ActionTrigger(TIMER_NUM, TIMER_ACTION_START);

.. _doxygen-group-list-section:

See Also
==========

相关 API Reference 请查看：

- :ref:`RAP <group___r_a_p>`
- :ref:`TIMER <group___t_i_m_e_r>`
- :ref:`IR <group___i_r>`
- :ref:`DMA <group___d_m_a>`
- :ref:`RCC <group___r_c_c>`
- :ref:`PINMUX <group___p_i_n_m_u_x>`
- :ref:`NVIC <group___n_v_i_c>`
