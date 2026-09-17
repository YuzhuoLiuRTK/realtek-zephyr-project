.. zephyr:code-sample:: bee-hid-mouse
   :name: Realtek Bee HID mouse (boot report + remote wakeup)
   :relevant-api: usbd_hid usbd_wakeup_request

   Combined HID mouse demo: boot/report protocol switch *and* remote wakeup on
   Realtek Bee SoCs.

Overview
********

This sample presents a USB HID mouse that combines the two HID features in one
device:

* **Boot interface support** (``protocol-code = "mouse"`` -> ``bInterfaceSubClass
  = 1``, ``bInterfaceProtocol = 2``). The device switches between:
    - *Boot Protocol* (``HID_PROTOCOL_BOOT = 0``): fixed 3-byte mouse report
      ``[buttons, X, Y]``, used by BIOS/UEFI pre-boot environments.
    - *Report Protocol* (``HID_PROTOCOL_REPORT = 1``): the 4-byte layout from
      the report descriptor, used by normal OSes.

  ``SET_PROTOCOL`` is handled by the usbd_hid class; this sample provides the
  ``set_protocol()`` callback to record the active protocol and re-formats
  every submitted report (and each ``GET_REPORT`` answer) into that layout.

* **Remote wakeup** (``CONFIG_SAMPLE_USBD_REMOTE_WAKEUP=y``). When the host
  suspends the USB bus and mouse activity occurs, the sample calls
  ``usbd_wakeup_request()`` to drive a bus resume (K state). A periodic
  ``PROBE`` line reports the stack-level suspend flags, so a lost USBSUSP
  interrupt can be told apart from a host that never suspended the bus.

* **DFU** (``CONFIG_USBD_DFU=y``). The DFU run-time instance is registered next
  to the HID interface, so the mouse advertises ``DFU_DETACH``. On detach the
  HID device is torn down and a separate DFU mode device (a different product
  ID, ``CONFIG_APP_DFU_MODE_PID``) is brought up, exposing two flash slots as
  alternate settings.

  The two slots come from ``src/dfu_slots.c``, a backend built directly on
  ``flash_area_*`` rather than on ``CONFIG_USBD_DFU_FLASH``. The in-tree
  backend writes through ``flash_img_*``, which resolves to a single
  compile-time area ID, so the alternate setting the host selects is ignored on
  download. Driving the flash map per image instead gives each alternate
  setting its own base and size straight from its partition:

  .. code-block:: none

     alt 0  ->  dfu_slot_a  (label "dfu-slot-a", string descriptor "slot-a")
     alt 1  ->  dfu_slot_b  (label "dfu-slot-b", string descriptor "slot-b")

  The alternate setting number is the position of the image in the linker
  section, which is sorted by the first ``USBD_DFU_DEFINE_IMG()`` argument,
  hence the ``a``/``b`` suffixes.

  Neither board layout defines slot-0/slot-1 partitions, so the board overlays
  shrink the application partition and carve the two slots out of the freed
  space: 2 x 256K on ``rtl87x2g_evb_a``, 2 x 100K on ``rtl87x2j_evb``. Both
  offsets are aligned to the respective flash erase-block size. Pages are
  erased lazily, only up to the byte the running download needs.

  Switching to DFU mode is one way: it tears down the HID device, so the mouse
  stops working and only comes back after a reboot. Nothing marks a downloaded
  image for boot either, the slots are plain storage.

Requirements
************

A board based on one of the supported Realtek Bee SoCs:

* :ref:`rtl87x2g_evb_a`
* :ref:`rtl87x2j_evb`

For the remote-wakeup part the host must explicitly allow the device to wake
it (e.g. *Allow this device to wake the computer* in Windows Device Manager),
otherwise ``usbd_wakeup_request()`` returns ``-EACCES``.

Building and Running
********************

.. zephyr-app-commands::
   :zephyr-app: samples/zephyr/drivers/usb/hid-mouse
   :board: rtl87x2j_evb_rtl8762jth
   :goals: build flash
   :compact:

Sample Output
=============

.. code-block:: console

   [00:00:00.xxx] <inf> main: HID mouse sample started (protocol=Report, rwup_armed=0)
   [00:00:00.xxx] <inf> main: USBD message: New device configuration [usbd=0 udc=1]
   [00:00:xx.xxx] <inf> main: HID protocol set to Boot
   [00:00:xx.xxx] <wrn> usbd_ch9: Set feature remote wakeup
   [00:00:xx.xxx] <inf> main: PROBE: bus state changed -> SUSPENDED [usbd=1 udc=1 rwup=1]
   [00:00:xx.xxx] <inf> main: Remote wakeup requested
   [00:00:xx.xxx] <inf> main: USBD message: Device resumed [usbd=0 udc=0]

DFU
===

While the mouse is running, the run-time interface answers ``DFU_DETACH``:

.. code-block:: console

   dfu-util -l
   dfu-util -d 2fe3:0001 --detach

The device then re-enumerates with the DFU mode product ID and two alternate
settings. Write and read back the second slot:

.. code-block:: console

   dfu-util -d 2fe3:ffff --alt 1 --download image.bin
   dfu-util -d 2fe3:ffff --alt 1 --upload readback.bin

.. code-block:: console

   [00:00:xx.xxx] <inf> main: USBD message: DFU detach request, DFU mode must be entered
   [00:00:xx.xxx] <inf> main: Detach USB device, switching to DFU mode
   [00:00:xx.xxx] <inf> main: HID device HID0 interface is not ready
   [00:00:xx.xxx] <inf> main: USBD message: New device configuration [usbd=0 udc=0]
   [00:00:xx.xxx] <inf> dfu_slots: area 5 download finished, 4096 bytes
