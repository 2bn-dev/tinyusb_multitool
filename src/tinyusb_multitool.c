
#define PICO_STDOUT_MUTEX 0 // STDOUT mutex causes deadlock with USB mutex during flash writes or anywhere that printf can happen within mutex from both cores.

#include "tusb.h"
#include "tinyusb_multitool.h"
#include "pico/mutex.h"
#include "hardware/irq.h"
#include "pico/stdio/driver.h"
#include "tumt_usb_config.h"
#include "tumt_uart_bridge.h"
#include <stdlib.h>

static mutex_t tumt_usb_mutex;

#define TUMT_USB_TASK_INTERVAL_US 1000
#define TUMT_USB_LOW_PRIORITY_IRQ 31

static mutex_t tumt_stdio_out_mutex;
static tumt_stdio_data_out_t *tumt_stdio_data_out[TUMT_STDIO_QUEUE_LENGTH];
static uint32_t tumt_stdio_data_out_read = 0;
static uint32_t tumt_stdio_data_out_write = 0;

static mutex_t tumt_stdio_in_mutex;
static tumt_stdio_data_out_t *tumt_stdio_data_in[TUMT_STDIO_QUEUE_LENGTH];
static uint32_t tumt_stdio_data_in_read = 0;
static uint32_t tumt_stdio_data_in_write = 0;

static_assert(TUMT_USB_LOW_PRIORITY_IRQ > RTC_IRQ, ""); // note RTC_IRQ is currently the last one


typedef void(*flash_pending_cb_t)(void);



void __no_inline_not_in_flash_func(tumt_periodic_task)(void) {
	//Non-blocking mutex, if it's already owned we'll hit it next time around.
	uint32_t interrupts = save_and_disable_interrupts();
	
	if (mutex_try_enter(&tumt_usb_mutex, NULL)){
		tud_task();
	        mutex_exit(&tumt_usb_mutex);
	}

	if(tumt_usb_stdio_connected())
		tumt_stdio_usb_out_chars();
	

	if(tumt_usb_uart0_connected())
		tumt_uart_bridge_uart0_in_out(&tumt_usb_mutex);
	
	if(tumt_usb_uart1_connected())
		tumt_uart_bridge_uart1_in_out(&tumt_usb_mutex);
	

	restore_interrupts(interrupts);
}

static int64_t __no_inline_not_in_flash_func(timer_task)(__unused alarm_id_t id, __unused void *user_data) {
	irq_set_pending(TUMT_USB_LOW_PRIORITY_IRQ);
	return TUMT_USB_TASK_INTERVAL_US;
}



static void __no_inline_not_in_flash_func(tumt_stdio_out_chars)(const char *buf, int length) {
	if(!tumt_usb_stdio_connected()) 
		return;
	while(length > 0){
		//length = TUMT_STDIO_MAX_STR_LEN;

		tumt_stdio_data_out_t *stdio_data;

		mutex_enter_blocking(&tumt_stdio_out_mutex);
		stdio_data = tumt_stdio_data_out[(tumt_stdio_data_out_write++)%TUMT_STDIO_QUEUE_LENGTH];
		mutex_exit(&tumt_stdio_out_mutex);

		if(length > TUMT_STDIO_MAX_STR_LEN){
			stdio_data->length = TUMT_STDIO_MAX_STR_LEN;
			memcpy(stdio_data->buf, buf, TUMT_STDIO_MAX_STR_LEN);
			length -= TUMT_STDIO_MAX_STR_LEN;
		}else{
			stdio_data->length = length;
			memcpy(stdio_data->buf, buf, length);
			length -= length;
		}
	}

	return;
}

void __no_inline_not_in_flash_func(tumt_stdio_usb_out_chars)(){
	static uint64_t last_avail_time;
	uint32_t owner;
	
	if (!mutex_try_enter(&tumt_usb_mutex, &owner)) {
		if (&owner != NULL && owner == get_core_num()) return; // would deadlock otherwise
		mutex_enter_blocking(&tumt_usb_mutex);
	}

	tumt_stdio_data_out_t *stdio_data;
	while(tumt_stdio_data_out_read < tumt_stdio_data_out_write){
		mutex_enter_blocking(&tumt_stdio_out_mutex);
		stdio_data = tumt_stdio_data_out[(tumt_stdio_data_out_read++)%TUMT_STDIO_QUEUE_LENGTH];
		mutex_exit(&tumt_stdio_out_mutex);
		if (tud_cdc_n_connected(CDCD_ITF_STDIO)) {
			for (int i = 0; i < stdio_data->length;) {
				int n = stdio_data->length - i;
				int avail = (int) tud_cdc_n_write_available(CDCD_ITF_STDIO);
				if (stdio_data->length > avail) 
					n = avail;
				if (n) {
					int n2 = (int) tud_cdc_n_write(CDCD_ITF_STDIO, stdio_data->buf+i, (uint32_t) n);
					tud_task();
					tud_cdc_n_write_flush(CDCD_ITF_STDIO);
					tud_task();
					i += n2;
					last_avail_time = time_us_64();
				} else {
					tud_task();
					tud_cdc_n_write_flush(CDCD_ITF_STDIO);
					tud_task();
					//if (!tud_cdc_n_connected(CDCD_ITF_STDIO) || (!tud_cdc_n_write_available(CDCD_ITF_STDIO) && time_us_64() > last_avail_time + TUMT_UART_USB_STDOUT_TIMEOUT_US)) {
						break; // For some reason during MSC reads cdc_n_write_available is almost always 0, this timeout is hit regularly?
					//}
				}
			}
		} else {
			// reset our timeout
			last_avail_time = 0;
		}
		//free(stdio_data); // Where we used to free memory, implementation now uses shared stdio_data array that exists forever to avoid mutex in malloc
	}

	mutex_exit(&tumt_usb_mutex);
}

static int __no_inline_not_in_flash_func(tumt_stdio_in_chars)(char *buf, int length) {
	return length;
}

int __no_inline_not_in_flash_func(tumt_stdio_usb_in_chars)() {
	uint32_t owner;
	if (!mutex_try_enter(&tumt_usb_mutex, &owner)) {
        	if (&owner != NULL && owner == get_core_num()) 
			return PICO_ERROR_NO_DATA; // would deadlock otherwise
        	mutex_enter_blocking(&tumt_usb_mutex);
	}

	int length;
	void* buf;
	int rc = PICO_ERROR_NO_DATA;
	if (tud_cdc_n_connected(CDCD_ITF_STDIO) && tud_cdc_n_available(CDCD_ITF_STDIO)) {
		int count = (int) tud_cdc_n_read(CDCD_ITF_STDIO, buf, sizeof(buf));
		rc =  count ? count : PICO_ERROR_NO_DATA;
	}
	
	mutex_exit(&tumt_usb_mutex);

	return rc;
}

stdio_driver_t tumt_stdio_usb = {
	.out_chars = tumt_stdio_out_chars,
	.in_chars = tumt_stdio_in_chars,
#if PICO_STDIO_ENABLE_CRLF_SUPPORT
	.crlf_enabled = TUMT_UART_USB_DEFAULT_CRLF
#endif
};

bool tumt_usb_init(void (*_flash_pending_cb)()) {
	for(uint8_t i = 0; i < TUMT_STDIO_QUEUE_LENGTH; i++){
		tumt_stdio_data_out[i] = malloc(sizeof(tumt_stdio_data_out_t)+1);
		tumt_stdio_data_in[i] = malloc(sizeof(tumt_stdio_data_out_t)+1);
	}

	// initialize TinyUSB
	tusb_init();

	irq_set_exclusive_handler(TUMT_USB_LOW_PRIORITY_IRQ, tumt_periodic_task);
	irq_set_enabled(TUMT_USB_LOW_PRIORITY_IRQ, true);

	mutex_init(&tumt_usb_mutex);
	mutex_init(&tumt_stdio_out_mutex);
	mutex_init(&tumt_stdio_in_mutex);

	stdio_set_driver_enabled(&tumt_stdio_usb, true);

	bool rc = add_alarm_in_us(TUMT_USB_TASK_INTERVAL_US, timer_task, NULL, true);
	return 1;
}

bool tumt_usb_stdio_connected(void) {
	return tud_cdc_n_connected(CDCD_ITF_STDIO);
}

bool tumt_usb_uart0_connected(void) {
	return tud_cdc_n_connected(CDCD_ITF_UART0);
}

bool tumt_usb_uart1_connected(void) {
	return tud_cdc_n_connected(CDCD_ITF_UART1);
}
