#include "tusb.h"
#include "tinyusb_multitool.h"
#include "pico/mutex.h"
#include "hardware/irq.h"
#include "pico/stdio/driver.h"
#include "tumt_usb_config.h"
#include "tumt_uart_bridge.h"
#include <stdlib.h>
#include "tumt_usb_stdio.h"

static mutex_t tumt_usb_mutex;

#define TUMT_USB_TASK_INTERVAL_US 1000
#define TUMT_USB_LOW_PRIORITY_IRQ 31

static_assert(TUMT_USB_LOW_PRIORITY_IRQ > RTC_IRQ, ""); // note RTC_IRQ is currently the last one


typedef void(*flash_pending_cb_t)(void);


mutex_t * tumt_get_usb_mutex(){
	return &tumt_usb_mutex;
}

void __no_inline_not_in_flash_func(tumt_periodic_task)(void) {
	//Non-blocking mutex, if it's already owned we'll hit it next time around.
	
	if(mutex_try_enter(&tumt_usb_mutex, NULL)){
		tud_task();

	if(tumt_usb_stdio_connected())
		tumt_stdio_usb_out_chars();
	if(tumt_usb_uart0_connected())
		tumt_uart_bridge_uart0_in_out(&tumt_usb_mutex);
	
	if(tumt_usb_uart1_connected())
		tumt_uart_bridge_uart1_in_out(&tumt_usb_mutex);

		mutex_exit(&tumt_usb_mutex);
	}
}

static int64_t __no_inline_not_in_flash_func(timer_task)(__unused alarm_id_t id, __unused void *user_data) {
	irq_set_pending(TUMT_USB_LOW_PRIORITY_IRQ);
	return TUMT_USB_TASK_INTERVAL_US;
}



bool tumt_usb_init(void (*_flash_pending_cb)()) {
	// initialize TinyUSB
	tusb_init();
	tumt_usb_stdio_init();

	irq_set_exclusive_handler(TUMT_USB_LOW_PRIORITY_IRQ, tumt_periodic_task);
	irq_set_enabled(TUMT_USB_LOW_PRIORITY_IRQ, true);

	mutex_init(&tumt_usb_mutex);

	bool rc = add_alarm_in_us(TUMT_USB_TASK_INTERVAL_US, timer_task, NULL, true);
	return 1;
}

bool tumt_usb_uart0_connected(void) {
	return tud_cdc_n_connected(CDCD_ITF_UART0);
}

bool tumt_usb_uart1_connected(void) {
	return tud_cdc_n_connected(CDCD_ITF_UART1);
}
