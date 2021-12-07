## TinyUSB Multitool

This repo attempts to combine the functionality of pico-stdio-usb with the USB functionality from the bootloader to present a USB mass storage device to load a bootloader, as well as adding the ability to flash an ESP32 over UART via the same mechanism. It also provides the ability to have additional USB UARTs, optionally bridged with physical UARTs.

The goal is to have a more extensible interface for both USB storage (msc) and USB serial ports (cdc) that isn't quite as complex as directly using tinyUSB.

### Supported hardware
 * Raspberry Pi 2040

### Attribution
 Portions of this software come from 

 * https://github.com/raspberrypi/pico-sdk
 * https://github.com/raspberrypi/pico-bootrom
 * https://github.com/raspberrypi/tinyusb

under the MIT or BSD-3-Clause license.
