

#define TUMT_UART_USB_DEFAULT_CRLF 1
#define TUMT_UART_USB_STDOUT_TIMEOUT_US 500000
#define TUMT_UART_USB_TASK_INTERVAL_US 10000
#define TUMT_UART_USB_LOW_PRIORITY_IRQ 31

#include "tusb.h"
#include "tinyusb_multitool.h"
#include "pico/mutex.h"
#include "hardware/irq.h"
#include "pico/stdio/driver.h"
#include "tumt_usb_config.h"
#include "tumt_uart_bridge.h"

static mutex_t tumt_usb_mutex;

#define TUMT_USB_TASK_INTERVAL_US 1000
#define TUMT_USB_LOW_PRIORITY_IRQ 31

static_assert(TUMT_USB_LOW_PRIORITY_IRQ > RTC_IRQ, ""); // note RTC_IRQ is currently the last one



mutex_t  tumt_get_usb_mutex(){
	return tumt_usb_mutex;
}

void __no_inline_not_in_flash_func(tumt_periodic_task)(void) {
	//Non-blocking mutex, if it's already owned we'll hit it next time around.
	if (mutex_try_enter(&tumt_usb_mutex, NULL)) {
		tud_task();
	        mutex_exit(&tumt_usb_mutex);
	}

	//tumt_uart_bridge_uart0_in_out(&tumt_usb_mutex);
	//tumt_uart_bridge_uart1_in_out();
}

static int64_t timer_task(__unused alarm_id_t id, __unused void *user_data) {
	irq_set_pending(TUMT_USB_LOW_PRIORITY_IRQ);
	return TUMT_USB_TASK_INTERVAL_US;
}



static void tumt_stdio_usb_out_chars(const char *buf, int length) {
    static uint64_t last_avail_time;
    uint32_t owner;
    if (!mutex_try_enter(&tumt_usb_mutex, &owner)) {
        if (owner == get_core_num()) return; // would deadlock otherwise
        mutex_enter_blocking(&tumt_usb_mutex);
    }
    if (tud_cdc_n_connected(USBD_ITF_CDC_STDIO)) {
        for (int i = 0; i < length;) {
            int n = length - i;
            int avail = (int) tud_cdc_n_write_available(USBD_ITF_CDC_STDIO);
            if (n > avail) n = avail;
            if (n) {
                int n2 = (int) tud_cdc_n_write(USBD_ITF_CDC_STDIO, buf + i, (uint32_t)n);
                tud_task();
                tud_cdc_n_write_flush(USBD_ITF_CDC_STDIO);
                i += n2;
                last_avail_time = time_us_64();
            } else {
                tud_task();
                tud_cdc_n_write_flush(USBD_ITF_CDC_STDIO);
                if (!tud_cdc_n_connected(USBD_ITF_CDC_STDIO) ||
                    (!tud_cdc_n_write_available(USBD_ITF_CDC_STDIO) && time_us_64() > last_avail_time + TUMT_UART_USB_STDOUT_TIMEOUT_US)) {
                    break;
                }
            }
        }
    } else {
        // reset our timeout
        last_avail_time = 0;
    }
    mutex_exit(&tumt_usb_mutex);
}

int tumt_stdio_usb_in_chars(char *buf, int length) {
    uint32_t owner;
    if (!mutex_try_enter(&tumt_usb_mutex, &owner)) {
        if (owner == get_core_num()) return PICO_ERROR_NO_DATA; // would deadlock otherwise
        mutex_enter_blocking(&tumt_usb_mutex);
    }
    int rc = PICO_ERROR_NO_DATA;
    if (tud_cdc_n_connected(USBD_ITF_CDC_STDIO) && tud_cdc_n_available(USBD_ITF_CDC_STDIO)) {
        int count = (int) tud_cdc_n_read(USBD_ITF_CDC_STDIO, buf, (uint32_t) length);
        rc =  count ? count : PICO_ERROR_NO_DATA;
    }
    mutex_exit(&tumt_usb_mutex);
    return rc;
}

stdio_driver_t tumt_stdio_usb = {
    .out_chars = tumt_stdio_usb_out_chars,
    .in_chars = tumt_stdio_usb_in_chars,
#if PICO_STDIO_ENABLE_CRLF_SUPPORT
    .crlf_enabled = TUMT_UART_USB_DEFAULT_CRLF
#endif
};

bool tumt_usb_init(void) {
	// initialize TinyUSB
	tusb_init();

	irq_set_exclusive_handler(TUMT_USB_LOW_PRIORITY_IRQ, tumt_periodic_task);
	irq_set_enabled(TUMT_USB_LOW_PRIORITY_IRQ, true);

	mutex_init(&tumt_usb_mutex);

	stdio_set_driver_enabled(&tumt_stdio_usb, true);

	bool rc = add_alarm_in_us(TUMT_USB_TASK_INTERVAL_US, timer_task, NULL, true);
	return 1;
}

bool tumt_usb_stdio_connected(void) {
	return tud_cdc_n_connected(USBD_ITF_CDC_STDIO);
}

bool tumt_usb_uart0_connected(void) {
	return tud_cdc_n_connected(USBD_ITF_CDC_UART0);
}

bool tumt_usb_uart1_connected(void) {
	return tud_cdc_n_connected(USBD_ITF_CDC_UART1);
}
