==================================
TIMER Timeout Trigger IR RX START
==================================
该示例演示了如何通过 :term:`RAP` (Real Autonomous Peripheral) 机制，利用 TIMER 的超时事件同步触发以下动作，实现无需 CPU 干预的红外信号收发协同测试：

1. **启动 PWM 输出**：作为红外发射源 (Carrier Waveform)。
2. **启动 IR 接收**：开始捕捉红外信号。
3. **启动 DMA 传输**：自动将 IR 接收到的数据搬运到内存。

随后，利用另一个 TIMER 超时事件通过 RAP 自动停止 PWM 输出，模拟红外信号的结束。CPU 仅需在红外接收完成（Idle Timeout）时处理数据。

主要流程：

1. **触发启动 (TIMER1_CH1 Timeout)**:

   * 启动 PWM 输出 (10kHz, 50% 占空比)。
   * 启动 IR 接收器。
   * 启动 DMA 通道进行数据搬运。
   * 启动停止计时器 (TIMER1_CH2)。

2. **自动停止 (TIMER1_CH2 Timeout)**:

   * 1秒后，停止 PWM 输出。

3. **数据处理 (IR Idle Interrupt)**:

   * 当红外信号停止后，IR 模块检测到空闲超时，触发中断。
   * CPU 读取剩余数据并打印接收到的波形数据。

环境需求
========
该示例的环境需求，可参考 :ref:`环境需求 <general_requirements_cn>`。

硬件连线
==============
本示例使用同一块开发板进行自发自收测试，请进行如下连接：

* **信号连接**: 将 **P0_1** (PWM 输出 / TX 模拟) 连接到 **P0_0** (IR 输入 / RX)。
* **GND**: 确保共地。

配置选项
==============
1. 可配置如下宏修改 PWM (发射源) 参数。

   .. code-block:: c

    #define PWM_OUT_PIN                         P0_1
    #define PWM_OUT_PERIOD                      4000  /* 10kHz @ 40MHz */
    #define PWM_OUT_HIGH_COUNT                  2000  /* 50% Duty Cycle */

2. 可配置如下宏修改 IR RX 参数。

   .. code-block:: c

    #define IR_RX_PIN                           P0_0
    #define IR_RX_DMA_WATERLEVEL                4     /* FIFO Level to trigger DMA */

3. 可配置如下宏修改发射持续时间。

   .. code-block:: c

    /* Stop Carrier Timer Period: 1 Second */
    #define TIMER_STOP_CARRIER_PERIOD           40000000

编译和下载
==========
该示例的编译和下载流程，可参考 :ref:`编译和下载 <general_build_download_cn>`。


测试验证
==========================
1. 按照“硬件连线”章节连接好引脚。
2. 启动开发板。
3. 观察串口日志，系统将触发一次红外收发过程，并打印接收到的数据：
   
   * **Log Output**:
     ::
       Start RAP timer timeout trigger IR RX start sample
       ...
       IR_INT_RX_CNT_THR (IR RX Complete)
       IR RX Length: ...
       IR RX Data[0]: 0x...
       IR RX Data[1]: 0x...

代码介绍
=======================
该章节主要介绍示例中的初始化和相应功能实现的代码和流程说明。

源码路径
--------

工程文件和源码路径如下：

* 工程路径: :file:`sdk\\sample\\peripheral\\rap\\timer_trigger_ir_dma\\proj`
* 源码路径: :file:`sdk\\sample\\peripheral\\rap\\timer_trigger_ir_dma\\src`

初始化
------
1. **PWM 初始化**: 配置 TIMER1_CH0 为 PWM 模式，输出 10kHz 方波。
2. **IR 初始化**: 配置为 RX 模式，使能 DMA 请求，设置 Idle Timeout 阈值。
3. **DMA 初始化**: 配置 DMA 通道从 IR FIFO 读取数据到内存 buffer。
4. **TIMER 初始化**:
   * **Start Carrier Timer**: 用于产生起始触发信号。
   * **Stop Carrier Timer**: 用于延时 1 秒后产生停止触发信号。

功能实现
--------
RAP 的配置在 ``main`` 函数中实现，使用了两个 RAP 通道：

1. **通道 0 (启动流程)**:
   * **源事件**: Start Carrier Timer 超时。
   * **动作**: 
     1. 启动 Stop Carrier Timer (开始 1秒倒计时)。
     2. 启动 PWM 输出。
     3. 启动 IR 接收。
     4. 启动 DMA 传输。

2. **通道 1 (停止流程)**:
   * **源事件**: Stop Carrier Timer 超时。
   * **动作**:
     1. 停止 PWM 输出 (模拟信号结束)。
     2. 停止 Stop Carrier Timer 自身。

   .. code-block:: c

    /* Channel 0: Start Sequence */
    RAP_EventRouteSet(TIMER_START_CARRIER_EVENT_TIMEOUT, channel0);
    RAP_ActionBindSet(TIMER_STOP_CARRIER_ACTION_START, channel0);
    RAP_ActionBindSet(PWM_OUT_ACTION_START, channel0);
    RAP_ActionBindSet(RAP_ACTION_IR_START_RX, channel0);
    RAP_ActionBindSet(IR_RX_DMA_ACTION_TRANSFER, channel0);

    /* Channel 1: Stop Sequence */
    RAP_EventRouteSet(TIMER_STOP_CARRIER_EVENT_TIMEOUT, channel1);
    RAP_ActionBindSet(TIMER_STOP_CARRIER_ACTION_STOP, channel1);
    RAP_ActionBindSet(PWM_OUT_ACTION_STOP, channel1);

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
