=====================================
RTC Compare Trigger KEYSCAN Sample
=====================================
该示例演示了如何通过 :term:`RAP` (Real Autonomous Peripheral) 机制，利用 RTC 的周期性比较事件触发 KEYSCAN 进行手动扫描，实现低功耗的按键检测方案。

其工作流程如下：
1. **初始状态**：KEYSCAN 配置为按键触发模式。当检测到按键按下时，CPU 唤醒并进入中断。
2. **扫描状态**：在首次检测到按键按下后，系统切换为 RTC 触发模式。RTC 通过 RAP 周期性触发 KEYSCAN 进行扫描，直到所有按键释放。
3. **结束状态**：当检测到所有按键释放后，关闭 RTC 和 RAP，重新将 KEYSCAN 配置回按键触发模式，等待下一次按键事件。

环境需求
========
该示例的环境需求，可参考 :ref:`环境需求 <general_requirements_cn>`。

硬件连线
==============
本示例配置为 2x2 矩阵键盘，连接如下：

* **行 (Row)**: P0_4, P0_5
* **列 (Column)**: P0_6, P0_7

配置选项
==============
1. 可配置如下宏修改 KEYSCAN 的行列引脚及大小。

   .. code-block:: c

    #define KEYBOARD_ROW_SIZE               2
    #define KEYBOARD_COLUMN_SIZE            2
    #define KEYBOARD_ROW_0                  P0_4
    /* ... other pins ... */

2. 可配置如下宏修改 RTC 的扫描周期（去抖动/轮询间隔）。

   .. code-block:: c

    #define RTC_PSC_VALUE                   (320 - 1)   /* 10ms tick */
    #define RTC_COMP_VALUE                  (5)         /* 50ms initial wait */
    #define RTC_COMP_RELOAD_VALUE           (5)         /* 50ms interval */

编译和下载
==========
该示例的编译和下载流程，可参考 :ref:`编译和下载 <general_build_download_cn>`。


测试验证
==========================
1. 启动开发板。
2. 按下矩阵键盘上的任意按键。
3. 串口助手将输出按键检测日志：
   ::
     KEYSCAN First Key Press Detected
     KEYSCAN One Key Press Detected: (0, 1) ...
   
4. 持续按住按键，日志会周期性输出检测结果（由 RTC 触发）。
5. 释放按键，串口输出释放信息，扫描停止：
   ::
     KEYSCAN: All Keys Released

代码介绍
=======================
该章节主要介绍示例中的初始化和相应功能实现的代码和流程说明。

源码路径
--------

工程文件和源码路径如下：

* 工程路径: :file:`sdk\\sample\\peripheral\\rap\\keyscan_trigger\\proj`
* 源码路径: :file:`sdk\\sample\\peripheral\\rap\\keyscan_trigger\\src`

初始化
------
1. 调用 ``board_keyscan_init`` 初始化键盘引脚。
2. 调用 ``driver_keyscan_init`` 初始化 KEYSCAN 外设，初始模式设为 :cpp:any:`KEYSCAN_MANUAL_SEL_KEY`（物理按键触发）。
3. 调用 ``driver_rtc_init`` 初始化 RTC，设定 50ms 的定时周期。

   .. code-block:: c

    static void driver_keyscan_init(KEYSCANManualSel_TypeDef Manual_Sel)
    {
        /* ... Clock Enable ... */
        KEYSCAN_InitStruct.KEYSCAN_ScanMode   = KEYSCAN_MANUAL_SCAN_MODE;
        KEYSCAN_InitStruct.KEYSCAN_DetectMode = KEYSCAN_DETECT_MODE_EDGE;
        KEYSCAN_InitStruct.KEYSCAN_ManualSel  = Manual_Sel; /* Key or Register Bit */
        KEYSCAN_Init(KEYSCAN, &KEYSCAN_InitStruct);
        /* ... Interrupt Configuration ... */
    }

功能实现
--------
逻辑主要分为 RAP 绑定与状态机切换（在 KEYSCAN 中断中处理）：

1. **RAP 绑定**：在 ``main`` 函数中，将 RTC 比较事件绑定到 RAP 通道，并关联两个动作：触发 KEYSCAN 手动扫描 和 RTC 自身重载。
   
   .. code-block:: c

    RAP_EventRouteSet(RTC_EVENT_COMPARE, channel0);
    RAP_ActionBindSet(RAP_ACTION_KEYSCAN_MANUAL, channel0);
    RAP_ActionBindSet(RTC_ACTION_RELOAD, channel0);

2. **状态切换 (中断处理)**：
   
   * **首次按下**：在中断中检测到按键后，将 KEYSCAN 触发源切换为寄存器位触发 (:cpp:any:`KEYSCAN_MANUAL_SEL_BIT`)，开启 RTC 和 RAP 模式。此时开始由 RTC 周期性触发扫描。
   
   * **持续按下**：RTC 每隔 50ms 触发一次扫描，KEYSCAN 完成扫描后产生中断，读取 FIFO 数据。
   
   * **按键释放**：当检测到 FIFO 为空时，停止 RTC，关闭 RAP 模式，并将 KEYSCAN 重新初始化为物理按键触发模式，等待下一次唤醒。

   .. code-block:: c

    void KEYSCAN_Handler(void)
    {
        /* ... Read FIFO ... */
        if (KEYSCAN_GetFlagState(KEYSCAN, KEYSCAN_FLAG_EMPTY) != SET) {
            if (is_first_pressed == false) {
                /* Switch to Periodic Scan Mode via RTC RAP */
                KEYSCAN_SetManualSelect(KEYSCAN, KEYSCAN_MANUAL_SEL_BIT);
                RTC_RAPModeCmd(ENABLE);
                KEYSCAN_RAPModeCmd(KEYSCAN, ENABLE);
                RTC_ActionTrigger(RTC_ACTION_START);
            }
        } else {
            /* All Keys Released: Stop RTC, Disable RAP, Reset to Key Trigger */
            RTC_ActionTrigger(RTC_ACTION_STOP);
            /* ... Disable RAP Modes ... */
            driver_keyscan_init(KEYSCAN_MANUAL_SEL_KEY);
        }
    }

.. _doxygen-group-list-section:

See Also
==========

相关 API Reference 请查看：

- :ref:`RAP <group___r_a_p>`
- :ref:`RTC <group___r_t_c>`
- :ref:`KEYSCAN <group___k_e_y_s_c_a_n>`
- :ref:`RCC <group___r_c_c>`
- :ref:`PINMUX <group___p_i_n_m_u_x>`
- :ref:`NVIC <group___n_v_i_c>`
