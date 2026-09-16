====================
General Introduction
====================
The document primarily introduces an overview of the peripheral sample, 
covering its :ref:`Requirements<general_requirements_en>`, :ref:`Configuration<general_config_en>`, 
:ref:`Building and Downloading<general_build_download_en>`, :ref:`Experimental Verification<general_exper_en>` and :ref:`Code Overview<general_code_overview_en>`. 

The document offers a detailed and comprehensive guide, covering every aspect from environment setup to code implementation, 
to help developers quickly get started with and test the peripheral sample.

.. _general_requirements_en:

Requirements
============

The sample supports the following development kits:

.. csv-table:: Development Kits
  :header: Hardware Platforms,Board Target
  :widths: 20 20
  :align: center

  RTL87x2J HDK, ``rtl87x2j_evb/rtl8762jth``

.. _general_config_en:

Configuration
==============
Users can refer to the Configuration in each peripheral sample for more detailed configuration information, including function configuration and pin definitions.

.. _general_build_download_en:

Building and Downloading
========================

The sample can be found in the repository folder:

* Project path: :file:`samples/peripheral/xxx/xxx`

To build and run the sample, follow the steps listed below:

#. Build the sample with ``west build``, specifying the board target:

   .. code-block:: shell

      west build -b rtl87x2j_evb/rtl8762jth samples/peripheral/xxx/xxx

#. After a successful build, flash the generated firmware onto the EVB with ``west flash``:

   .. code-block:: shell

      west flash

#. Press the :kbd:`reset` button on the EVB board and it will start running.

.. _general_exper_en:

Experimental Verification
==========================
Users can refer to the Experimental Verification in each peripheral sample for more detailed verification flow and result.

.. _general_code_overview_en:

Code Overview
==============
This section introduces the source code directory and initialization, including the peripheral initialization and a description of the functionality implementation flow.

.. _general_source_code_dir_en:

Source Code Directory
----------------------

This section describes the project directory and structure. The directory for project file and source code are as follows:

* Project directory: :file:`samples/peripheral/xxx/xxx`
* Source code directory: :file:`samples/peripheral/xxx/xxx/src`

The project is built with Zephyr's standard CMake + Kconfig build system, and the directory structure is as follows:

.. highlight:: rst

::

   └── xxx                                            sample category, such as gpio
       └── xxx                                        sample name, such as output_toggle
           ├── CMakeLists.txt                         sample CMake build script
           ├── prj.conf                               sample Kconfig configuration
           └── src
               └── xxx_xxx.c                          sample application source file, such as gpio_output_toggle.c


.. _general_init_procedure_en:

Initialization
---------------

When the system is powered on or reset, the ``main`` function is called to execute the following initialization functions:

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

Peripheral initialization mainly consists of ``board_xxx_init()`` and ``driver_xxx_init()`` functions. 

- The ``board_xxx_init()`` is responsible for PAD and :term:`PINMUX` configuration.

- The ``driver_xxx_init()`` handles clock configuration, peripheral initialization parameter configuration, interrupt configuration and enabling the peripherals etc. 

Details about application sample descriptions are provided in each peripheral sample.

.. _general_peripheral_init_flow_en:

Initialization Flow
~~~~~~~~~~~~~~~~~~~
Below is the common flow for peripheral initialization. 
While different peripherals generally follow this common flow, any specific variations for certain peripherals will be explained in their respective sample.

Peripheral initialization mainly consists of the following components:

* Configure PAD and PINMUX.
* Enable peripheral clock.
* Initialize peripheral.
* Configure :term:`NVIC` & Enable peripheral interrupt if necessary.
* Enable peripheral.

The initialization flow is shown in the following figure, where 'XXX' is the name of the peripheral being initialized, such as :term:`GPIO`, :term:`I2C`, or :term:`SPI`.

.. figure:: ../figures/peripheral_init_flow.*
   :align: center
   :name: Peripheral Initialization Flow
 
   Peripheral Initialization Flow


PAD Configuration
~~~~~~~~~~~~~~~~~~~
Call the :cpp:any:`Pad_Config` function to configure software mode or pinmux mode, pull up or pull down or pull none, output or input, and output high or low level.

.. code-block:: c

   Pad_Config(P0_5, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_HIGH);


PINMUX Configuration
~~~~~~~~~~~~~~~~~~~~~
Call the :cpp:any:`Pinmux_Config` function to select :cpp:any:`PAD_PINMUX_MODE`, and call the :cpp:any:`Pinmux_Config` function to select the peripheral function, such as :c:macro:`DWGPIO`, only then will the pin have the peripheral capability.

.. code-block:: c

   /* Configure Pin P0_5 as GPIO function */
   Pad_Config(P0_5, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_HIGH);
   Pinmux_Config(P0_5, DWGPIO);

.. note::
   It is forbidden to set different PADs as the same peripheral function (except DWGPIO) at the same time.
   For example, setting P0_0 and P0_1 as UART0_TX simultaneously is not allowed.

Clock Configuration
~~~~~~~~~~~~~~~~~~~~~
Before initializing the peripherals, it is necessary to enable the peripherals clock. 
Call the :cpp:any:`RCC_ClockCmd` function to enable the peripherals clock.

.. code-block:: c

   RCC_ClockCmd(GPIO_CLOCK, ENABLE);

Initialize Peripheral
~~~~~~~~~~~~~~~~~~~~~
When initializing a peripheral, define an initialization structure and configure its parameters according to your needs to achieve the desired functionality. 
Then call the ``XXX_Init`` function to initialize the peripheral.

.. code-block:: c

   GPIO_InitTypeDef GPIO_InitStruct;
   GPIO_StructInit(&GPIO_InitStruct);
   GPIO_InitStruct.GPIO_Pin        = GPIO_PIN;
   GPIO_InitStruct.GPIO_Mode       = GPIO_DIR_IN;
   ...
   GPIO_Init(GPIO_PORT, &GPIO_InitStruct);

.. _general_nvic_config_en:

Interrupt Configuration
~~~~~~~~~~~~~~~~~~~~~~~~
Call the :cpp:any:`NVIC_Init` function to enable :term:`IRQ` interrupt.

.. code-block:: c

   NVIC_InitTypeDef NVIC_InitStruct;
   NVIC_InitStruct.NVIC_IRQChannel = GPIO5_IRQ;
   NVIC_InitStruct.NVIC_IRQChannelPriority = 5;
   NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStruct);

.. note::
   User IRQ priority should be set  between 2 and 6 (
   priorities 0 and 1 are reserved for interrupts with very high real-time requirements and may be affected by the :term:`OS`; 
   priority 7 is reserved for system PendSV/SysTick).


Enable Peripheral
~~~~~~~~~~~~~~~~~~~~~
Call the ``XXX_Cmd`` function to enable corresponding peripheral feature.

.. code-block:: c

   ADC_Cmd(ADC, ADC_ONE_SHOT_MODE, ENABLE);


Deinitialize Peripheral
~~~~~~~~~~~~~~~~~~~~~~~~
Call the ``XXX_DeInit`` function to deinitializes the peripheral registers to their default values.

.. code-block:: c

   GPIO_DeInit(GPIOA);
