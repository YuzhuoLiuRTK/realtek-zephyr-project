================================
TIMER Timeout Trigger I2C Write
================================
This example demonstrates how to use the :term:`RAP` (Real Autonomous Peripheral) mechanism to trigger an I2C Master Write Transaction via a TIMER timeout event, without CPU intervention for the start sequence.

In this example, the I2C Master is configured in **Wrapper Mode**. This mode allows pre-configuration of the transaction type (Write mode in this case) and the data to be sent. When a RAP event (such as a TIMER timeout) occurs, the hardware automatically executes the complete I2C Write timing sequence.

Key Workflow:
1. **Pre-configuration**: Configure I2C Master in Wrapper Mode, set to Write Mode, and pre-fill the TX FIFO.
2. **Trigger**: The TIMER generates a timeout event every 1 second, triggering the I2C Master to start the transaction via RAP.
3. **Transaction**: The I2C Master automatically sends 10 bytes of data (0, 1, ... 9).
4. **Processing**: The CPU only reads data in the interrupt service routine when the Slave RX FIFO is full or a Stop signal is detected, and sets a flag to notify the main function. The main function then prints the data.

Requirements
=============
For requirements, please refer to the :ref:`Requirements <general_requirements_en>`.

Wiring
==============
This example uses two I2C interfaces on the same EVB for communication testing. Connect as follows:

* **SCL Connection**: Connect **P0_0** (I2C0 Master SCL) to **P0_4** (I2C1 Slave SCL).
* **SDA Connection**: Connect **P0_1** (I2C0 Master SDA) to **P0_5** (I2C1 Slave SDA).
* **GND**: Ensure common ground.

Configurations
==============
1. The following macros can be configured to modify I2C parameters.

   .. code-block:: c

    #define I2C_SPEED                   100000  /* 100kHz */
    #define I2C_SLAVE_ADDR              0x50
    #define I2C_MASTER_WRITE_LEN        10      /* Master writes 10 bytes */

2. The following macros can be configured to modify the TIMER trigger period.

   .. code-block:: c

    #define TIMER_PERIOD                40000000 /* 1 Second */

Building and Downloading
========================
For building and downloading, please refer to the :ref:`Building and Downloading <general_build_download_en>`.

Experimental Verification
==========================
1. Connect the pins as described in the "Wiring" section.
2. Start the EVB.
3. Observe the serial log. The system will perform I2C communication every 1 second:
   
   * **Slave Log**: Prints the received data length and content.
     ::
       I2C_SLAVE: I2C_INT_RX_FULL
       I2C_SLAVE: I2C_INT_STOP_DET
       I2C Slave RX Length = 10
       Master TX Data[0]: 0, Slave RX Data[0]: 0
       Master TX Data[1]: 1, Slave RX Data[1]: 1
       ...
       Master TX Data[9]: 9, Slave RX Data[9]: 9

Code Overview
=======================
This section introduces the code and process description for initialization and corresponding function implementation in the sample.

Source Code Directory
----------------------

The directory for project file and source code are as follows:

* Project directory: :file:`sdk\\sample\\peripheral\\rap\\timer_trigger_i2c_write\\proj`
* Source code directory: :file:`sdk\\sample\\peripheral\\rap\\timer_trigger_i2c_write\\src`

Initialization
--------------
1. **I2C Slave Init**: Configure as Slave mode, enable :c:macro:`I2C_INT_RX_FULL` and :c:macro:`I2C_INT_STOP_DET` interrupts.
2. **I2C Master Init**:
   
   * Call ``driver_i2c_master_init`` for basic configuration.
   * Call ``driver_i2c_master_wrapper_config`` to enable **Wrapper Mode**:
     
     * :cpp:any:`I2C_WrapperModeCmd`: Enable Wrapper function.
     * :cpp:any:`I2C_WrapperSetTransMode`: Set mode to :cpp:any:`I2C_WRAPPER_TRANS_MODE_WRITE`.
     * Pre-fill Write FIFO.

   .. code-block:: c

    static void driver_i2c_master_wrapper_config(void)
    {
        /* ... Data Init ... */
        
        /* Configure I2C Wrapper Mode */
        I2C_WrapperModeCmd(I2C_MASTER, ENABLE);
        I2C_WrapperSetTransMode(I2C_MASTER, I2C_WRAPPER_TRANS_MODE_WRITE);
        I2C_WrapperClearTxFIFO(I2C_MASTER);
        I2C_WrapperSetWriteNum(I2C_MASTER, I2C_MASTER_WRITE_LEN);
        I2C_WrapperSetWriteData(I2C_MASTER, i2c_master_tx_buffer, I2C_MASTER_WRITE_LEN);
    }

Functional Implementation
--------------------------
The RAP configuration is implemented in the ``main`` function:

1. **RAP Routing**:
   
   * Route the TIMER Timeout event (``TIMER_EVENT_TIMEOUT``) to a RAP channel.
   * Bind the I2C Start action (:cpp:any:`I2C_ACTION_START`) to this channel.

2. **Start**:
   * Enable RAP mode for TIMER and I2C.
   * Start the TIMER. I2C Write transactions will subsequently be triggered by hardware timing.

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

3. **Main Function Processing**:
   After I2C Slave receives data, the interrupt function sets the flag ``is_receive_done`` to notify the main function. After receiving the notification, the main function prints the received data length and content, and verifies the data correctness.

   .. code-block:: c

       while (1)
       {
           if (is_receive_done)
           {
               /* Clear flag immediately */
               is_receive_done = false;

               /* Print received data information */
               DBG_DIRECT("I2C Slave RX Length = %d", i2c_slave_rx_length);

               /* Verify and print data */
               for (uint32_t i = 0; i < i2c_slave_rx_length; i++)
               {
                   if (i2c_master_tx_buffer[i] != i2c_slave_rx_buffer[i])
                   {
                       DBG_DIRECT("Data[%d]: 0x%X (Expected: 0x%X) - ERROR",
                                  i, i2c_slave_rx_buffer[i], i2c_master_tx_buffer[i]);
                   }
                   else
                   {
                       DBG_DIRECT("Data[%d]: 0x%X - OK", i, i2c_slave_rx_buffer[i]);
                   }
               }
           }
       }

.. _doxygen-group-list-section:

See Also
==========

Please refer to the relevant API Reference:

- :ref:`RAP <group___r_a_p>`
- :ref:`TIMER <group___t_i_m_e_r>`
- :ref:`I2C <group___i2_c>`
- :ref:`RCC <group___r_c_c>`
- :ref:`PINMUX <group___p_i_n_m_u_x>`
- :ref:`NVIC <group___n_v_i_c>`
