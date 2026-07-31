.. zephyr:code-sample:: usb-audio-uac-mic
   :name: USB Audio microphone
   :relevant-api: _usb_device_core_api

   Implement a USB Audio microphone (UAC IN) device that captures from an
   I2S codec on RTL87x2G.

Overview
********

This sample demonstrates a USB Audio Class microphone device on RTL87x2G.
Audio is captured from an external codec over I2S RX and streamed to the
USB host over the ISO IN endpoint. Only the microphone (capture / IN) path
is implemented.

Data flow:

- The host selects the streaming interface (alt=1), which triggers
  ``data_request_cb``. The app starts the I2S + codec and primes the first
  USB frame.
- An I2S reader thread continuously reads codec samples into a shared buffer.
- ``data_written_cb`` chains the next USB IN transfer after each completion,
  sending captured I2S data (or silence until the first block arrives).

Building and Running
********************

An overlay describing the USB, I2S and codec hardware is required. By
default ``app.overlay`` (targeting the RTL87x2G I2S0 + internal codec) is
applied.

After building and flashing to your board, plug the board into a USB host.

Testing
*******

Steps to test the sample:

- Build and flash the sample as described above.
- Connect the board to the host.
- Choose the device as the default audio input (microphone).
- Start recording (for example using Audacity).
- Verify the recorded audio stream.
