#ifdef _PICO_STDIO_USB_H
#error TinyUSB multitool is incompatible with pico-stdio-usb, they cant be used together
#endif


#ifndef TINYUSB_MULTITOOL_H
#define TINYUSB_MULTITOOL_H
#include "tusb.h"
#include "tumt_debug.h"
#include "tumt_uart_bridge.h"


#define TUMT_STDIO_MAX_STR_LEN 128
#define TUMT_STDIO_QUEUE_LENGTH 64

#define TUMT_UART_USB_DEFAULT_CRLF 1
#define TUMT_UART_USB_STDOUT_TIMEOUT_US 500000 // 500 ms - Disabled
#define TUMT_UART_USB_TASK_INTERVAL_US 1000 // currently unused
#define TUMT_UART_USB_LOW_PRIORITY_IRQ 31

struct {
	int length;
	char * buf[TUMT_STDIO_MAX_STR_LEN+1];
} typedef tumt_stdio_data_out_t;

bool tumt_uart_usb_connected(void);
void tumt_periodic_task(void);
bool tumt_usb_init(void (*_flash_pending_cb)(void));
mutex_t tumt_get_usb_mutex(void);
bool tumt_usb_stdio_connected(void);
bool tumt_usb_uart0_connected(void);
bool tumt_usb_uart1_connected(void);


static void __no_inline_not_in_flash_func(tumt_stdio_out_chars)(const char *buf, int length);
void __no_inline_not_in_flash_func(tumt_stdio_usb_out_chars)();
static int __no_inline_not_in_flash_func(tumt_stdio_in_chars)(char *buf, int length);
int __no_inline_not_in_flash_func(tumt_stdio_usb_in_chars)();
#endif
