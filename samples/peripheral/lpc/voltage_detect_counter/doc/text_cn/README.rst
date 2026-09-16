==================
Voltage Detect Counter
==================
该示例验证 :term:`LPC` 电压检测计数功能。

使用 P2_2 作为电压检测的引脚。当检测 P2_2 的输入电压高于设定阈值计数次数时会触发 LPC 中断。

.. note::
  - 可作为电压检测作用的引脚为 P2_0 ~ P2_7、Vbat，检测电压阈值范围：60mV~3600mV

环境需求
========
该示例的环境需求，可参考 :ref:`环境需求 <general_requirements_cn>`。

硬件连线
==============
连接 P2_2 和外部输入电压。

配置选项
==============

1. 可配置如下宏修改引脚定义。

   .. code-block:: c

    #define LPC_CAPTURE_PIN             P2_2
    #define LPC_CAPTURE_CHANNEL         LPC_CHANNEL_ADC2

2. 可配置如下宏修改 LPC 比较器的触发边沿设置。

   .. code-block:: c

    #define LPC_VOLTAGE_DETECT_EDGE         LPC_Vin_Over_Vth        /*< Set this macro to select the LPC detect edge. Selectable parameters include LPC_Vin_Over_Vth and LPC_Vin_Below_Vth. */

3. 可配置如下宏修改 LPC 比较器的触发阈值电压。

   .. code-block:: c

    #define LPC_COMPARE_VOLTAGE              LPC_1080_mV          /*< Configure LPC Threshold Voltage.*/

4. 可配置如下宏修改 LPC 比较器的触发次数。

   .. code-block:: c

    #define LPC_COMPARE_COUNTER              10                 /*< Interrupt will trigger when the counter reaches this value.*/


编译和下载
==========
该示例的编译和下载流程，可参考 :ref:`编译和下载 <general_build_download_cn>`。


测试验证
==========================

1. EVB 启动后，在 Debug Analyzer 工具内观察 log 。
   ::
     Start lpc counter sample

2. 当 P2_2 检测到输入电压高于 :cpp:any:`LPC_COMPARE_VOLTAGE`  电压次数为 :cpp:any:`LPC_COMPARE_COUNTER` 时，触发 :c:macro:`LPC_INT_COUNTER_COMPARE` 中断，在 Debug Analyzer 工具内打印进入电压检测计数器中断的 log 。
   ::
     LPC_Handler: Counter Reached Threshold: XXX


代码介绍
=======================
该章节主要介绍示例中的初始化和相应功能实现的代码和流程说明。

源码路径
--------

工程文件和源码路径如下：

* 工程路径: :file:`sdk\\sample\\peripheral\\lpc\\voltage_detect_counter\\proj`
* 源码路径: :file:`sdk\\sample\\peripheral\\lpc\\voltage_detect_counter\\src`

初始化
------
外设的初始化流程可参考 :Doc:`General Introduction <../../../../doc/general_introduction/text_cn/README>` 中的 :ref:`初始化流程<general_peripheral_init_flow_cn>` 部分。

1. 调用 :cpp:any:`Pad_Config` 与 :cpp:any:`Pinmux_Config`，配置对应引脚的 PAD 和 PINMUX。

   .. code-block:: c

    void board_lpc_init(void)
    {
        Pad_Config(LPC_TEST_PIN, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE, PAD_OUT_HIGH);
        Pinmux_Config(LPC_TEST_PIN, IDLE_MODE);
    }

2. 由于 LPC 位于 AON 区域，因此无需调用 :cpp:any:`RCC_ClockCmd`。
3. 对 LPC 外设进行初始化：

   a. 定义 :cpp:any:`LPC_InitTypeDef` 类型 ``LPC_InitStruct``，调用 :cpp:any:`LPC_StructInit` 将 ``LPC_InitStruct`` 预填默认值。
   b. 根据需求修改 ``LPC_InitStruct`` 参数，LPC 的初始化参数配置如下表。
   c. 调用 :cpp:any:`LPC_Init`，初始化 LPC 外设。

.. csv-table:: LPC 初始化参数
  :header: LPC Hardware Parameters, Setting in the ``LPC_InitStruct`` , LPC
  :widths: 40 40 40
  :align: center

  Channel, :cpp:any:`LPC_InitTypeDef::LPC_Channel`, :cpp:any:`LPC_CAPTURE_CHANNEL`
  Edge, :cpp:any:`LPC_InitTypeDef::LPC_Edge`, :cpp:any:`LPC_VOLTAGE_DETECT_EDGE`
  Threshold Voltage, :cpp:any:`LPC_InitTypeDef::LPC_Threshold`, :cpp:any:`LPC_COMPARE_VOLTAGE`

4. 调用 :cpp:any:`LPC_CounterReset`，复位LPC计数器。
5. 调用 :cpp:any:`LPC_SetComparator`，设置比较计数器阈值。
6. 调用 :cpp:any:`LPC_INTConfig`，配置 LPC 计数次数中断 :c:macro:`LPC_INT_COUNTER_COMPARE` 和 NVIC。 NVIC 相关配置可参考 :ref:`中断配置<general_nvic_config_cn>`。
7. 调用 :cpp:any:`LPC_CounterCmd` ，使能 LPC 计数比较。

.. _lpc_voltage_detection_function_cn:

功能实现
--------

1. 当 P2_2 检测到电压高于设置的计数器阈值时，触发 :c:macro:`LPC_INT_COUNTER_COMPARE` 中断，在中断函数内打印相关信息，清除中断标志位。

   .. code-block:: c

    void LPC_Handler(void)
    {
      /* Check if Counter Compare Interrupt occurred */
      if (LPC_GetINTStatus(LPC0, LPC_INT_COUNTER_COMPARE) == SET)
      {
         DBG_DIRECT("LPC_Handler: Counter Reached Threshold: %d", LPC_COMPARE_COUNTER);

         /* Disable the interrupt to prevent continuous triggering if the voltage */
         LPC_INTConfig(LPC0, LPC_INT_COUNTER_COMPARE, DISABLE);

         /* Clear the interrupt status */
         LPC_ClearINTStatus(LPC0, LPC_INT_COUNTER_COMPARE);
      }
     }


.. _doxygen-group-list-section:

See Also
==========

相关 API Reference 请查看：

- :ref:`LPC <group___l_p_c>`
- :ref:`PINMUX <group___p_i_n_m_u_x>`
- :ref:`RCC <group___r_c_c>`
- :ref:`NVIC <group___n_v_i_c>`