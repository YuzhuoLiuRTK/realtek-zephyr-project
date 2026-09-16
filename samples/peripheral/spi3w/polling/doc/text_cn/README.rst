==================
Polling
==================
该示例通过使用 :term:`SPI3W` 的轮询方式读取鼠标传感器 ID 信息。

通过 SPI3W 与鼠标传感器进行通信，通过读取指定地址下的数据来读取指定信息。

环境需求
========
该示例的环境需求，可参考 :ref:`环境需求 <general_requirements_cn>`。

硬件连线
==============
EVB 外接 PWM3610DM-SUDU 鼠标，连接 CLK，DATA 和 CS 引脚至鼠标模块，同时连接 GND 和 VDD。具体引脚配置详见 :ref:`配置选项<spi3w_polling_configuration_cn>`。

硬件介绍
---------
PWM3610DM-SUDU 为激光鼠标传感器模块。PMW3610DM-SUDU 寄存器可通过串行端口访问。寄存器用于读取运动数据和状态，以及设备配置等信息。
在此示例中，使用该模块用于演示与 SPI3W 的通信，用来读取该模块的 ID 信息。

.. _spi3w_polling_configuration_cn:

配置选项
==============

1. 可配置如下宏修改引脚定义。
 
   .. code-block:: c

    /* SPI3W Pin Configuration */
    #define SPI3W_CLK_PIN               P4_0
    #define SPI3W_DATA_PIN              P4_1
    #define SPI3W_CS_PIN                P4_2


编译和下载
==========
该示例的编译和下载流程，可参考 :ref:`编译和下载 <general_build_download_cn>`。


测试验证
==========================

1. EVB 启动后，SPI3W 开始与鼠标传感器进行通信。通信结束后，打印获取到的 ID 信息。若 ID 为 0x3E 和 0x01（仅代表 PWM3610DM-SUDU 鼠标），代表测试成功。
   ::
     SPI3W Read ID: id[0] = 0x3e, id[1] = 0x1
     SPI3W Read ID Pass


代码介绍
=======================
该章节主要介绍示例中的初始化和相应功能实现的代码和流程说明。

源码路径
--------

工程文件和源码路径如下：

* 工程路径: :file:`sdk\\sample\\peripheral\\spi3w\\polling\\proj`
* 源码路径: :file:`sdk\\sample\\peripheral\\spi3w\\polling\\src`

初始化
------
外设的初始化流程可参考 :Doc:`General Introduction <../../../../doc/general_introduction/text_cn/README>` 中的 :ref:`初始化流程<general_peripheral_init_flow_cn>` 部分。

1. 调用 :cpp:any:`Pad_Config` 与 :cpp:any:`Pinmux_Config`，配置对应引脚的 PAD 和 PINMUX。

   .. code-block:: c

    void board_spi3w_init(void)
    {
        Pad_Config(SPI3W_CLK_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE,
                   PAD_OUT_HIGH);
        Pad_Config(SPI3W_DATA_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE,
                   PAD_OUT_HIGH);
        Pad_Config(SPI3W_CS_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE,
                   PAD_OUT_HIGH);
    
        Pinmux_Config(SPI3W_CLK_PIN, SPI3W_CLK_MASTER);
        Pinmux_Config(SPI3W_DATA_PIN, SPI3W_DATA_MASTER);
        Pinmux_Config(SPI3W_CS_PIN, SPI3W_CS_MASTER);
    }

2. 调用 :cpp:any:`RCC_ClockCmd`，开启 SPI3W 时钟。
3. 对 SPI3W 外设进行初始化：

   a. 定义 :cpp:any:`SPI3W_InitTypeDef` 类型 ``SPI3W_InitStruct`` ，调用 :cpp:any:`SPI3W_StructInit` 将 ``SPI3W_InitStruct`` 预填默认值。
   b. 根据需求修改 ``SPI3W_InitStruct`` 参数，SPI3W 的初始化参数配置如下表。
   c. 调用 :cpp:any:`SPI3W_Init`，初始化 SPI3W 外设。

.. csv-table:: SPI3W 初始化参数
  :header: SPI3W Hardware Parameters, Setting in the ``SPI3W_InitStruct`` , SPI3W
  :widths: 40 40 40
  :align: center

  SPI3W Source Clock, :cpp:any:`SPI3W_InitTypeDef::SPI3W_SysClock`, 20000000
  SPI3W Clock, :cpp:any:`SPI3W_InitTypeDef::SPI3W_Speed`, 800000
  SPI3W Mode (3-Wire or 2-Wire), :cpp:any:`SPI3W_InitTypeDef::SPI3W_Mode`, :cpp:any:`SPI3W_3WIRE_MODE`
  Read Delay Cycle, :cpp:any:`SPI3W_InitTypeDef::SPI3W_ReadDelay`, 0x3

.. _spi3w_polling_function_cn:

功能实现
---------
SPI3W 以轮询模式读取数据的流程如图所示：

.. figure:: ../../../polling/doc/figures/spi3w_polling_read_flow.*
   :align: center
   :scale: 100%
   :alt: 这里应该是 SPI polling read flow
   :name: 图片-SPI polling read flow

   SPI3W 轮询模式读取数据流程图


1. 调用 ``spi3w_read_byte`` 函数，传入地址参数，读取鼠标 ID 信息。代码中使用 ``SPI3W_WAIT_WHILE`` 宏来处理状态等待和超时机制。
   
   a. 等待 busy 状态标志位消除。
   b. 调用 :cpp:any:`SPI3W_ClearRxDataLen`，通信开始前清除 SPI3W 的接收的数据长度。
   c. 调用 :cpp:any:`SPI3W_StartRead`，传入地址信息和需要读取的数据长度，开始读取该地址下的数据。等待 busy 状态标志位消除。
   d. 循环判断是否接收到数据。如果接收到数据，调用 :cpp:any:`SPI3W_ReadBuf` 读取 SPI3W 接收 FIFO 中数据。
   e. 返回读取的数据。

   .. code-block:: c

    uint8_t spi3w_read_byte(uint8_t address)
    {
        uint8_t reg_value = 0xFF;

        /* Check busy before reading */
        SPI3W_WAIT_WHILE(SPI3W_GetFlagStatus(SPI3W_FLAG_BUSY) == SET);

        /* Clear RX data length before reading */
        SPI3W_ClearRxDataLen();

        /* Start Read */
        SPI3W_StartRead(address, 1);

        /* Check read command is write succesfully */
        SPI3W_WAIT_WHILE(SPI3W_GetFlagStatus(SPI3W_FLAG_BUSY) == SET);

        /* Check RX FIFO has received data */
        SPI3W_WAIT_WHILE(SPI3W_GetRxDataLen() == 0);

        /* Read data from RX FIFO */
        SPI3W_ReadBuf(&reg_value, 1);

        return reg_value;
    }


.. _doxygen-group-list-section:

See Also
==========

相关 API Reference 请查看：

- :ref:`RCC <group___r_c_c>`
- :ref:`SPI3W <group___s_p_i3_w>`
- :ref:`PINMUX <group___p_i_n_m_u_x>`
