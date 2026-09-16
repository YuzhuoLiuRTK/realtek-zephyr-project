=====
概述
=====
该文档主要介绍了外设示例的概述，涵盖 :ref:`环境需求<general_requirements_cn>` 、:ref:`配置选项<general_config_cn>`、
:ref:`编译和下载<general_build_download_cn>`，:ref:`测试验证<general_exper_cn>` 和 :ref:`代码介绍<general_code_overview_cn>` 几个部分。

该文档提供了详细而全面的指南，从环境设置到代码实现的各个方面，帮助开发人员快速入门并测试外设示例。

.. _general_requirements_cn:

环境需求
==============
该示例支持以下开发套件：

.. csv-table:: 开发套件
  :header: Hardware Platforms,Board Target
  :widths: 20 20
  :align: center

  RTL87x2J HDK, ``rtl87x2j_evb/rtl8762jth``

.. _general_config_cn:

配置选项
==============
用户可以参考各外设示例的配置选项，以获取更详细的配置信息，包括实现功能，引脚定义等。

.. _general_build_download_cn:

编译和下载
============
示例的工程路径如下:

* 工程路径: :file:`samples/peripheral/xxx/xxx`

请按照以下步骤操作构建并运行该示例:

#. 使用 ``west build`` 编译该示例，并指定 board target：

   .. code-block:: shell

      west build -b rtl87x2j_evb/rtl8762jth samples/peripheral/xxx/xxx

#. 编译成功后，通过 ``west flash`` 将生成的固件下载到 EVB：

   .. code-block:: shell

      west flash

#. 按下 :kbd:`复位` 按键，开始运行。

.. _general_exper_cn:

测试验证
========
用户可以参考各外设示例的测试验证，以获取更详细的验证流程和结果。

.. _general_code_overview_cn:

代码介绍
=========
在该章节中，介绍源码路径和初始化，包括外设初始化和功能实现的流程。

.. _general_source_code_dir_cn:

源码路径
--------

该节介绍了工程路径和结构。工程文件和源码路径如下：

* 工程路径: :file:`samples/peripheral/xxx/xxx`
* 源码路径: :file:`samples/peripheral/xxx/xxx/src`

工程采用 Zephyr 标准的 CMake + Kconfig 构建，目录结构如下：

.. highlight:: rst

::

   └── xxx                                            sample category, such as gpio
       └── xxx                                        sample name, such as output_toggle
           ├── CMakeLists.txt                         sample CMake build script
           ├── prj.conf                               sample Kconfig configuration
           └── src
               └── xxx_xxx.c                          sample application source file, such as gpio_output_toggle.c


.. _general_init_procedure_cn:

初始化
------

当系统上电或复位时，会调用 ``main`` 来并执行以下初始化功能：

.. code-block:: c

   int main(void)
   {
      /* Enable Global Interrupts */
      __enable_irq();

      DBG_DIRECT("Start xxx sample");

      /* Peripheral xxx initialization */
      board_xxx_init();
      driver_xxx_init();

      while (1)
      {
          /* Sample code can be added here */
      }
   }
 

外设初始化主要涉及： ``board_xxx_init()`` 和 ``driver_xxx_init()``。

- ``board_xxx_init()`` 负责 PAD 与 :term:`PINMUX` 的配置。

- ``driver_xxx_init()`` 负责时钟配置，外设初始化参数配置，中断配置，启用外设等。

应用示例在各外设示例中进行详细说明。

.. _general_peripheral_init_flow_cn:

初始化流程
~~~~~~~~~~
下面展示了外设初始化的通用流程。不同外设的初始化通常遵循这一通用流程，个别外设会略有差异的情况会在外设示例中进行说明。

外设初始化主要包括以下流程：

*  配置外设 PAD 和 PINMUX。
*  启用外设时钟。
*  配置外设初始化参数，并初始化外设。
*  在必要时，配置 :term:`NVIC` 并启用外设中断。
*  启用外设。

初始化流程如下图所示，其中 “XXX” 是进行初始化的外设名称，例如 :term:`GPIO` 、:term:`I2C` 或 :term:`SPI`。

.. figure:: ../figures/peripheral_init_flow.*
   :align: center
   :name: Peripheral Initialization Flow
 
   外设初始化流程图

PAD 配置
~~~~~~~~~~
可以通过调用 :cpp:any:`Pad_Config` 函数来配置软件模式或复用模式，电阻上拉或下拉或浮空，输出或输入，输出高或低。

.. code-block:: c

   Pad_Config(P0_5, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_HIGH);

PINMUX 配置
~~~~~~~~~~~~
调用 :cpp:any:`Pad_Config` 函数来选择 :cpp:any:`PAD_PINMUX_MODE`, 调用 :cpp:any:`Pinmux_Config` 函数来选择外设功能，例如 :c:macro:`DWGPIO`，该引脚才具备外设功能。

.. code-block:: c

   /* Configure Pin P0_5 as GPIO function */
   Pad_Config(P0_5, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_HIGH);
   Pinmux_Config(P0_5, DWGPIO);

.. note::
   禁止将不同的 PAD 同时设置为相同的外设功能（DWGPIO 除外）。例如，同时将 P0_0 和 P0_1 设置为 UART0_TX 是不允许的。

时钟配置
~~~~~~~~~~~~
在初始化外设之前，需要启用外设时钟。调用 :cpp:any:`RCC_ClockCmd` 函数来启用外设时钟。

.. code-block:: c

   RCC_ClockCmd(GPIO_CLOCK, ENABLE);


外设初始化
~~~~~~~~~~~~
在初始化外设时，定义初始化结构体，根据需要配置结构体的参数来实现所需功能，调用 ``XXX_Init`` 函数来初始化外设。

.. code-block:: c

   GPIO_InitTypeDef GPIO_InitStruct;
   GPIO_StructInit(&GPIO_InitStruct);
   GPIO_InitStruct.GPIO_Pin        = GPIO_PIN;
   GPIO_InitStruct.GPIO_Mode       = GPIO_DIR_IN;
   ...
   GPIO_Init(GPIO_PORT, &GPIO_InitStruct);

.. _general_nvic_config_cn:

中断配置
~~~~~~~~~~~~
调用 :cpp:any:`NVIC_Init` 函数以启用 :term:`IRQ` 中断。

.. code-block:: c

   NVIC_InitTypeDef NVIC_InitStruct;
   NVIC_InitStruct.NVIC_IRQChannel = GPIO5_IRQ;
   NVIC_InitStruct.NVIC_IRQChannelPriority = 5;
   NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStruct);

.. note::
   用户 IRQ 优先级应设置为 2 ~ 6（0 ~ 1 优先级用于实时性要求非常高的中断，此优先级受 :term:`OS` 影响；7 优先级为系统 PendSV/SysTick）。

外设使能
~~~~~~~~~~~~
调用 ``XXX_Cmd`` 函数，使能相应外设功能。

.. code-block:: c

   ADC_Cmd(ADC, ADC_ONE_SHOT_MODE, ENABLE);

去初始化
~~~~~~~~~~~~
调用 ``XXX_DeInit`` 函数，将外设寄存器去初始化为默认值。

.. code-block:: c

   GPIO_DeInit(GPIOA);
