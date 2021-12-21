#define CFG_TUD_CDC_FLUSH_ON_SOF 1
#include "tusb.h"
#include "tinyusb_multitool.h"
#include "pico/mutex.h"
#include "pico/stdio.h"
#include "pico/stdio/driver.h"
#include "tumt_usb_config.h"
#include "tumt_usb_stdio.h"
#include <stdlib.h>

critical_section_t tumt_stdio_out_critical_section;
static tumt_stdio_data_out_t *tumt_stdio_data_out[TUMT_STDIO_QUEUE_LENGTH];
static uint32_t tumt_stdio_data_out_read = 0;
static uint32_t tumt_stdio_data_out_write = 0;

critical_section_t tumt_stdio_in_critical_section;
static tumt_stdio_data_out_t *tumt_stdio_data_in[TUMT_STDIO_QUEUE_LENGTH];
static uint32_t tumt_stdio_data_in_read = 0;
static uint32_t tumt_stdio_data_in_write = 0;



static void __no_inline_not_in_flash_func(tumt_stdio_out_chars)(const char *buf, int length) {
	if(!tumt_usb_stdio_connected() || length == 0) 
		return;

	uint32_t i = 0;
	
	uint32_t attempts = 0;
	while(tumt_stdio_data_out_write-tumt_stdio_data_out_read >= TUMT_STDIO_QUEUE_LENGTH-1 && attempts < 25){
		tumt_stdio_usb_out_chars();
		if(attempts > 0){
			busy_wait_ms(1);
		}
		attempts++;
	}
	if(attempts >= 25)
		return; // data loss
	
	critical_section_enter_blocking(&tumt_stdio_out_critical_section);
	while(length > 0){
		tumt_stdio_data_out_t *stdio_data;
		stdio_data = tumt_stdio_data_out[(tumt_stdio_data_out_write++)%TUMT_STDIO_QUEUE_LENGTH];

		if(length >= TUMT_STDIO_MAX_STR_LEN){
			stdio_data->length = TUMT_STDIO_MAX_STR_LEN;
			memcpy(stdio_data->buf, buf+(i*(TUMT_STDIO_MAX_STR_LEN)), TUMT_STDIO_MAX_STR_LEN*sizeof(char));
			length -= TUMT_STDIO_MAX_STR_LEN;
			i++;
		}else{
			stdio_data->length = length;
			memcpy(stdio_data->buf, buf+(i*(TUMT_STDIO_MAX_STR_LEN)), length*sizeof(char));
			length = 0;
		}
	}

	critical_section_exit(&tumt_stdio_out_critical_section);
	return;
}

void __no_inline_not_in_flash_func(tumt_stdio_usb_out_chars)(){
	static uint64_t last_avail_time;
	tumt_stdio_data_out_t *stdio_data;

	critical_section_enter_blocking(&tumt_stdio_out_critical_section);
	while(tumt_stdio_data_out_read < tumt_stdio_data_out_write){
		if (tud_cdc_n_connected(CDCD_ITF_STDIO)) {
			stdio_data = tumt_stdio_data_out[(tumt_stdio_data_out_read++)%TUMT_STDIO_QUEUE_LENGTH];
			int i;
			for (i = 0; i < stdio_data->length;) {
				int n = stdio_data->length - i;
				int avail = (int) tud_cdc_n_write_available(CDCD_ITF_STDIO);
				if (n > avail) 
					n = avail;
				if (n) {
					int n2 = (int) tud_cdc_n_write(CDCD_ITF_STDIO, stdio_data->buf+i, (uint32_t) n);
					tud_cdc_n_write_flush(CDCD_ITF_STDIO);
					i += n2;
					last_avail_time = time_us_64();
				} else {
					tud_cdc_n_write_flush(CDCD_ITF_STDIO);

					if (!tud_cdc_n_connected(CDCD_ITF_STDIO) || (!tud_cdc_n_write_available(CDCD_ITF_STDIO) /*&& time_us_64() > last_avail_time + TUMT_UART_USB_STDOUT_TIMEOUT_US*/)) {
						break; // For some reason during MSC reads cdc_n_write_available is almost always 0, this timeout is hit regularly?
					}
				}
			}
			
			if(i < stdio_data->length){
				uint8_t temp_buf[TUMT_STDIO_MAX_STR_LEN];

				memcpy(&temp_buf, stdio_data->buf+i, TUMT_STDIO_MAX_STR_LEN-i);
				memcpy(stdio_data->buf, &temp_buf, (stdio_data->length - i));
				
				stdio_data->length = stdio_data->length - i;
				tumt_stdio_data_out_read--;
				break;
			}
		} else {
			// reset our timeout
			last_avail_time = 0;
		}
	}

	if(tumt_stdio_data_out_read == tumt_stdio_data_out_write){
		tumt_stdio_data_out_read = 0;
		tumt_stdio_data_out_write = 0;
	}

	critical_section_exit(&tumt_stdio_out_critical_section);
}

static int __no_inline_not_in_flash_func(tumt_stdio_in_chars)(char *buf, int length) {
	return length;
}

int __no_inline_not_in_flash_func(tumt_stdio_usb_in_chars)() {
	int length;
	void* buf;
	int rc = PICO_ERROR_NO_DATA;
	if (tud_cdc_n_connected(CDCD_ITF_STDIO) && tud_cdc_n_available(CDCD_ITF_STDIO)) {
		int count = (int) tud_cdc_n_read(CDCD_ITF_STDIO, buf, sizeof(buf));
		rc =  count ? count : PICO_ERROR_NO_DATA;
	}
	
	return rc;
}

stdio_driver_t tumt_stdio_usb = {
	.out_chars = tumt_stdio_out_chars,
	.in_chars = tumt_stdio_in_chars,
#if PICO_STDIO_ENABLE_CRLF_SUPPORT
	.crlf_enabled = TUMT_UART_USB_DEFAULT_CRLF
#endif
};

bool tumt_usb_stdio_init() {
	for(uint8_t i = 0; i < TUMT_STDIO_QUEUE_LENGTH; i++){
		tumt_stdio_data_out[i] = malloc(sizeof(tumt_stdio_data_out_t)+1);
		tumt_stdio_data_out[i]->buf = malloc((TUMT_STDIO_MAX_STR_LEN+1)*sizeof(char));
		memset(tumt_stdio_data_out[i]->buf, 0x00, TUMT_STDIO_MAX_STR_LEN);
		tumt_stdio_data_in[i] = malloc(sizeof(tumt_stdio_data_out_t)+1);
		tumt_stdio_data_in[i]->buf = malloc((TUMT_STDIO_MAX_STR_LEN+1)*sizeof(char));
		memset(tumt_stdio_data_in[i]->buf, 0x00, TUMT_STDIO_MAX_STR_LEN);
	}

	critical_section_init(&tumt_stdio_out_critical_section);
	critical_section_init(&tumt_stdio_in_critical_section);

	stdio_set_driver_enabled(&tumt_stdio_usb, true);

	return 1;
}

bool tumt_usb_stdio_connected() {
	return tud_cdc_n_connected(CDCD_ITF_STDIO);
}
