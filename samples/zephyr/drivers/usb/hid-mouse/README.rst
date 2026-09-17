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
