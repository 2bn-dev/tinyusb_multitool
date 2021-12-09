#ifdef _PICO_STDIO_USB_H
#error TinyUSB multitool is incompatible with pico-stdio-usb, they cant be used together
#endif


#ifndef TINYUSB_MULTITOOL_H
#define TINYUSB_MULTITOOL_H
#include "tusb.h"
#include "tinyusb_multitool_debug.h"


bool tumt_uart_usb_connected(void);
void tumt_periodic_task(void);
bool tumt_usb_init(void);
mutex_t tumt_get_usb_mutex(void);

#endif
