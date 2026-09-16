======================================
TIMER Timeout Trigger I2C Repeat Read
======================================
该示例演示了如何通过 :term:`RAP` (Real Autonomous Peripheral) 机制，利用 TIMER 的超时事件触发 I2C Master 发起一次“重复读” (Repeat Read) 传输，无需 CPU 介入启动过程。

在本示例中，I2C Master 被配置为 **Wrapper 模式**。这种模式允许预先配置好传输类型（如 Write-Read）和数据，当 RAP 事件（如 TIMER 超时）到达时，硬件自动执行完整的 I2C 时序。

主要流程：
1. **预配置**: I2C Master 配置为 Wrapper 模式，设定为 Repeat Read (先写后读)，并预填发送 FIFO。
2. **触发**: TIMER 每隔 1 秒产生超时事件，通过 RAP 触发 I2C Master 启动传输。
3. **传输**: I2C Master 自动发送 10 字节数据，随后发送 Repeated Start 信号，再读取 24 字节数据。
4. **处理**: CPU 仅在 Slave 收到请求或 Master 传输完成（Stop Detect）时，在中断中处理数据。

环境需求
========
该示例的环境需求，可参考 :ref:`环境需求 <general_requirements_cn>`。

硬件连线
==============
本示例使用同一块开发板上的两个 I2C 接口进行通信测试，请进行如下连接：

* **SCL 连接**: 将 **P0_0** (I2C0 Master SCL) 连接到 **P0_4** (I2C1 Slave SCL)。
* **SDA 连接**: 将 **P0_1** (I2C0 Master SDA) 连接到 **P0_5** (I2C1 Slave SDA)。
* **GND**: 确保共地。

配置选项
==============
1. 可配置如下宏修改 I2C 参数。

   .. code-block:: c

    #define I2C_SPEED                   100000  /* 100kHz */
    #define I2C_SLAVE_ADDR              0x50
    #define I2C_MASTER_WRITE_LEN        10      /* Master 先写 10 字节 */
    #define I2C_MASTER_READ_LEN         24      /* Master 再读 24 字节 */

2. 可配置如下宏修改 TIMER 的触发周期。

   .. code-block:: c

    #define TIMER_PERIOD                40000000 /* 1 Second */

编译和下载
==========
该示例的编译和下载流程，可参考 :ref:`编译和下载 <general_build_download_cn>`。


测试验证
==========================
1. 按照“硬件连线”章节连接好引脚。
2. 启动开发板。
3. 观察串口日志，系统将每隔 1 秒进行一次 I2C 通信：
   
   * **Slave Log**: 打印接收到的数据（Master 写入的 0~9）和发送的数据长度。
   * **Master Log**: 打印读取到的数据（Slave 返回的 16~39）。
     ::
       I2C_SLAVE: I2C_INT_RX_FULL
       I2C_SLAVE: I2C_INT_RD_REQ
       I2C Slave RX Length: 10
       I2C Slave RX Data[0]: 0
       ...
       I2C_MASTER: I2C_INT_STOP_DET
       I2C Master RX Data[0]: 16
       ...

代码介绍
=======================
该章节主要介绍示例中的初始化和相应功能实现的代码和流程说明。

源码路径
--------

工程文件和源码路径如下：

* 工程路径: :file:`sdk\\sample\\peripheral\\rap\\timer_trigger_i2c\\proj`
* 源码路径: :file:`sdk\\sample\\peripheral\\rap\\timer_trigger_i2c\\src`

初始化
------
1. **I2C Slave 初始化**: 配置为从机模式，开启 :c:macro:`I2C_INT_RX_FULL` (接收缓冲满) 和 :c:macro:`I2C_INT_RD_REQ` (读请求) 中断。
2. **I2C Master 初始化**:
   
   * 调用 ``driver_i2c_master_init`` 进行基础配置。
   * 调用 ``driver_i2c_master_wrapper_config`` 启用 **Wrapper 模式**：
     
     * :cpp:any:`I2C_WrapperModeCmd`: 使能 Wrapper 功能。
     * :cpp:any:`I2C_WrapperSetTransMode`: 设置模式为 :cpp:any:`I2C_WRAPPER_TRANS_MODE_REPEAT_READ`。
     * 预填充 Write FIFO 和配置 Read 长度。

   .. code-block:: c

    static void driver_i2c_master_wrapper_config(void)
    {
        /* ... Data Init ... */
        I2C_WrapperModeCmd(I2C_MASTER, ENABLE);
        I2C_WrapperSetTransMode(I2C_MASTER, I2C_WRAPPER_TRANS_MODE_REPEAT_READ);

        /* Configure Write Phase */
        I2C_WrapperSetWriteNum(I2C_MASTER, I2C_MASTER_WRITE_LEN);
        I2C_WrapperSetWriteData(I2C_MASTER, i2c_master_tx_buffer, I2C_MASTER_WRITE_LEN);

        /* Configure Read Phase */
        I2C_WrapperSetReadNum(I2C_MASTER, I2C_MASTER_READ_LEN);
        
        I2C_Cmd(I2C_MASTER, ENABLE);
    }

功能实现
--------
RAP 的配置在 ``main`` 函数中实现：

1. **RAP 路由**:
   
   * 将 TIMER 超时事件 (``TIMER_EVENT_TIMEOUT``) 路由到 RAP 通道。
   * 将 I2C 启动动作 (:cpp:any:`I2C_ACTION_START`) 绑定到该通道。

2. **启动**:
   * 开启 TIMER 和 I2C 的 RAP 模式。
   * 启动 TIMER，之后 I2C 传输将由硬件定时触发。

   .. code-block:: c

    /* Route TIMER Timeout Event to RAP channel */
    RAP_EventRouteSet(TIMER_EVENT_TIMEOUT, channel);
    /* Bind I2C Start Action to RAP channel */
    RAP_ActionBindSet(I2C_ACTION_START, channel);

    /* Enable RAP Mode */
    TIMER_RAPModeCmd(TIMER_NUM, ENABLE);
    I2C_RAPModeCmd(I2C_MASTER, ENABLE);

    /* Start TIMER */
    TIMER_ActionTrigger(TIMER_NUM, TIMER_ACTION_START);

.. _doxygen-group-list-section:

See Also
==========

相关 API Reference 请查看：

- :ref:`RAP <group___r_a_p>`
- :ref:`TIMER <group___t_i_m_e_r>`
- :ref:`I2C <group___i2_c>`
- :ref:`RCC <group___r_c_c>`
- :ref:`PINMUX <group___p_i_n_m_u_x>`
- :ref:`NVIC <group___n_v_i_c>`
