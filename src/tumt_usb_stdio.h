#ifdef _PICO_STDIO_USB_H
#error TinyUSB multitool USB STDIO is incompatible with pico-stdio-usb, they cant be used together
#endif


#ifndef TUMT_USB_STDIO_H
#define TUMT_USB_STDIO_H
#include "tusb.h"
#include "tumt_debug.h"

#define TUMT_STDIO_MAX_STR_LEN 128
#define TUMT_STDIO_QUEUE_LENGTH 32

#define TUMT_UART_USB_STDOUT_TIMEOUT_US 500000 // 500 ms - Disabled

struct {
	int length;
	char * buf;
} typedef tumt_stdio_data_out_t;

bool tumt_usb_stdio_init();
bool tumt_usb_stdio_connected(void);


static void __no_inline_not_in_flash_func(tumt_stdio_out_chars)(const char *buf, int length);
void __no_inline_not_in_flash_func(tumt_stdio_usb_out_chars)();
static int __no_inline_not_in_flash_func(tumt_stdio_in_chars)(char *buf, int length);
int __no_inline_not_in_flash_func(tumt_stdio_usb_in_chars)();
#endif
