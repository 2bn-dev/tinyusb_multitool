#ifdef _PICO_STDIO_USB_H
#error TinyUSB multitool is incompatible with pico-stdio-usb, they cant be used together
#endif


#ifndef TINYUSB_MULTITOOL_H
#define TINYUSB_MULTITOOL_H

#include "tusb.h"
#include "tumt_debug.h"
#include "tumt_uart_bridge.h"

#define TUMT_UART_USB_DEFAULT_CRLF 1
#define TUMT_UART_USB_STDOUT_TIMEOUT_US 500000 // 500 ms - Disabled
#define TUMT_UART_USB_TASK_INTERVAL_US 1000 // currently unused
#define TUMT_UART_USB_LOW_PRIORITY_IRQ 31

mutex_t *tumt_get_usb_mutex(void);
void tumt_periodic_task(void);
bool tumt_usb_init(void (*_flash_pending_cb)(void));

bool tumt_usb_uart0_connected(void);
bool tumt_usb_uart1_connected(void);
#endif
