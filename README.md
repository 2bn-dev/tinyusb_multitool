## TinyUSB Multitool

## WARNING: This is still VERY WIP

This repo attempts to combine the functionality of pico-stdio-usb with the USB functionality from the bootloader to present a USB mass storage device to load a bootloader, as well as adding the ability to flash an ESP32 over UART via the same me
chanism. It also provides the ability to have additional USB UARTs, optionally bridged with physical UARTs.

The goal is to have a more extensible interface for both USB storage (msc) and USB serial ports (cdc) that isn't quite as complex as directly using tinyUSB.


### Features (planned/implemented)

- [x] USB Mass storage flash interface similar to bootloader
- [x] Flashing RP2040 via USB MSC
- [ ] Loading RP2040 RAM only image via USB MSC
- [x] UART_Bridge ESP32 ESP_SYNC detection and enter ESP32 flash mode via GPIO.
- [ ] Flashing ESP32 via USB MSC over UART0
- [x] STDIO to USB Serial port
- [x] UART Bridge uart0 to USB Serial port
- [x] UART Bridge uart1 to USB Serial port
- [ ] I2C0/1 Bridge to USB Serial Port
- [ ] Increased configurability (USB Mass storage on/off, Serial on/off for each)
- [ ] Custom USB interface extensibility?
- [ ] WebUSB?
- [ ] USB Mass storage READ flash file
- [ ] USB Mass storage more runtime info
- [ ] USB Mass storage README.txt
- [ ] USB Mass storage - Add custom files / file handler
- [ ] USB Mass storage -> ESP32 webserver dynamic IP load
- [ ] RP2040 Stage 2.5/3 bootloader for Flash partition failover (OTA1/OTA2/FALLBACK) if OTA1 = active flash OTA2 boot OTA2 if fail boot OTA1, if fail boot FALLBACK (fallback can't be OTA flashed).
- [ ] Flash file system direct USB mass storage read/write?


### Supported hardware
 * Raspberry Pi 2040

### Attribution
 Portions of this software come from 

 * https://github.com/raspberrypi/pico-sdk
 * https://github.com/raspberrypi/pico-bootrom
 * https://github.com/raspberrypi/tinyusb

under the MIT or BSD-3-Clause license.
