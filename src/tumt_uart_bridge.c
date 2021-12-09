
#include <stdlib.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "hardware/uart.h"
#include "device/usbd_pvt.h"
#include "tinyusb_multitool.h"
#include "tumt_usb_config.h"
#include "tumt_uart_bridge.h"


const char ESP_SYNC[] = { 
	0xc0, 0x00, 0x08, 0x24, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x07, 0x07, 0x12, 0x20, 0x55, 0x55, 0x55,
       	0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 
	0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 
	0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 
	0x55, 0x55, 0x55, 0x55, 0x55, 0xC0
};

uint32_t current_baud_rate_uart0 = 0;
uint32_t current_baud_rate_uart1 = 0;

static int64_t tumt_uart_bridge_uart0_timer(__unused alarm_id_t id, __unused void *user_data) {
        tumt_uart_bridge_uart0_in_out(tumt_get_usb_mutex());
        return TUMT_UART_BRIDGE_TASK_INTERVAL_US;
}

void tumt_uart_bridge_uart0_init(uint32_t baud_rate){
	current_baud_rate_uart0 = uart_init(uart0, baud_rate);
	uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
	uart_set_translate_crlf(uart0, false);
	bool rc = add_alarm_in_us(TUMT_UART_BRIDGE_TASK_INTERVAL_US, tumt_uart_bridge_uart0_timer, NULL, true);
}


static int64_t tumt_uart_bridge_uart1_timer(__unused alarm_id_t id, __unused void *user_data) {
        tumt_uart_bridge_uart1_in_out(tumt_get_usb_mutex());
        return TUMT_UART_BRIDGE_TASK_INTERVAL_US;
}
void tumt_uart_bridge_uart1_init(uint32_t baud_rate){
	current_baud_rate_uart1 = uart_init(uart1, baud_rate);
	uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
	uart_set_translate_crlf(uart1, false);

	bool rc = add_alarm_in_us(TUMT_UART_BRIDGE_TASK_INTERVAL_US, tumt_uart_bridge_uart1_timer, NULL, true);
}

void tumt_uart_bridge_uart0_deinit(){
	uart_deinit(uart0);
	current_baud_rate_uart0 = 0;
}

void tumt_uart_bridge_uart1_deinit(){
	uart_deinit(uart1);
	current_baud_rate_uart1 = 0;
}
void tumt_uart_bridge_pin_init(void){
	gpio_set_function(PIN_TXD0, GPIO_FUNC_UART);
        gpio_set_function(PIN_RXD0, GPIO_FUNC_UART);
	gpio_set_function(PIN_TXD1, GPIO_FUNC_UART);
	gpio_set_function(PIN_RXD1, GPIO_FUNC_UART);


	//TODO: IFDEF
        gpio_init(PIN_ESP32_FLASH);
        gpio_init(PIN_ESP32_EN);

        gpio_pull_down(PIN_ESP32_FLASH);
	sleep_ms(1);
	gpio_pull_up(PIN_ESP32_EN);
	sleep_ms(1);
	gpio_pull_down(PIN_ESP32_EN);
	//This resets ESP32
}



/* From ESPTOOL:
 * # issue reset-to-bootloader:
 * # RTS = either CH_PD/EN or nRESET (both active low = chip in reset
 * # DTR = GPIO0 (active low = boot to flasher)
 * #
 * # DTR & RTS are active low signals,
 * # ie True = pin @ 0V, False = pin @ VCC.
 *
 * (Note: 2bn LED MCU uses inverter for EN and FLASH pins between RP2040 and ESP32).
 *
 * gpio_pull_up(PIN_ESP32_FLASH); == bootloader mode
 * gpio_pull_up(PIN_ESP32_EN);    == chip disable
 * gpio_pull_down(PIN_ESP32_EN);  == chip enable
 */


/* Sequencing from ESPTOOL (seconds):
 * DTR: 0.0000000 False == TINYUSB 0 ?
 * RTS: 0.0007728 True == TINYUSB 1
 * DTR: 0.1010019 True 
 * RTS: 0.1040874 False
 * DTR: 0.1542244 False
 */



//Read buffer for ESP32 Sync identification
char read_buf_uart0[128] = {0};
int32_t read_buf_write_uart0 = 0;
bool match_found_uart0 = false;

int tumt_uart_bridge_uart0_in_out(mutex_t tumt_usb_mutex){
    	static uint64_t last_avail_time;
	uint32_t owner;

	if (!mutex_try_enter(&tumt_usb_mutex, &owner)) {
		if (owner == get_core_num()) return 0; // would deadlock otherwise
		mutex_enter_blocking(&tumt_usb_mutex);
	}

	if (!tud_cdc_n_connected(USBD_ITF_CDC_UART0)) {
		mutex_exit(&tumt_usb_mutex);
		return 0;
	}

	while(uart_is_readable(uart0) && tud_cdc_n_connected(USBD_ITF_CDC_UART0) && tud_cdc_n_write_available(USBD_ITF_CDC_UART0) > 0){
		int32_t buf = uart_getc(uart0);
		if(buf >= 0){
			tud_cdc_n_write_char(USBD_ITF_CDC_UART0, buf);
	                tud_task();
	                tud_cdc_n_write_flush(USBD_ITF_CDC_UART0);
			tud_task();
	                last_avail_time = time_us_64();
		}
	}

	//uart_tx_wait_blocking(uart0);
	while(uart_is_writable(uart0) && tud_cdc_n_connected(USBD_ITF_CDC_UART0) && tud_cdc_n_available(USBD_ITF_CDC_UART0)){
		int32_t buf;
		 buf = tud_cdc_n_read_char(USBD_ITF_CDC_UART0);
		if(buf < 0){
			break;
		}
		uart_putc_raw(uart0, buf);


		read_buf_uart0[read_buf_write_uart0] = buf;
		read_buf_write_uart0++;
		if(read_buf_write_uart0 >= sizeof(read_buf_uart0)){
			read_buf_write_uart0 = 0;
			
		}

		if(!match_found_uart0){
			for(int32_t i = 0; i < sizeof(read_buf_uart0); i++){
				if(read_buf_uart0[i] == 0xC0){
					int32_t i_orig = i;
					int32_t ii = 0;
					for(; ii < sizeof(ESP_SYNC); ii++){
						if(read_buf_uart0[i] != ESP_SYNC[ii]){
							break;
						}else{
							i++;
							if(i >= sizeof(read_buf_uart0)){
								i = 0;
							}
						}
					}
					if(ii == sizeof(ESP_SYNC)-1){
						//ESP Sync magic found, enter flash mode
						gpio_pull_up(PIN_ESP32_FLASH);
						gpio_pull_up(PIN_ESP32_EN);
						sleep_ms(1);
						gpio_pull_down(PIN_ESP32_EN);
						match_found_uart0 = true;
						//TODO: We need to flip this back after ESP32 Image
						break;
					}else{
						i = i_orig;
					}
				}
			}
		}
	}

	mutex_exit(&tumt_usb_mutex);
	return 0;
}

//Read buffer for ESP32 Sync identification
char read_buf_uart1[128] = {0};
int32_t read_buf_write_uart1 = 0;
bool match_found_uart1 = false;

int tumt_uart_bridge_uart1_in_out(mutex_t tumt_usb_mutex){
        static uint64_t last_avail_time;
        uint32_t owner;

        if (!mutex_try_enter(&tumt_usb_mutex, &owner)) {
                if (owner == get_core_num()) return 0; // would deadlock otherwise
                mutex_enter_blocking(&tumt_usb_mutex);
        }

        if (!tud_cdc_n_connected(USBD_ITF_CDC_UART1)) {
                mutex_exit(&tumt_usb_mutex);
                return 0;
        }

        while(uart_is_readable(uart1) && tud_cdc_n_connected(USBD_ITF_CDC_UART1) && tud_cdc_n_write_available(USBD_ITF_CDC_UART1) > 0){
                int32_t buf = uart_getc(uart1);
                if(buf >= 0){
                        tud_cdc_n_write_char(USBD_ITF_CDC_UART1, buf);
                        tud_task();
                        tud_cdc_n_write_flush(USBD_ITF_CDC_UART1);
                        tud_task();
                        last_avail_time = time_us_64();
                }
        }

        //uart_tx_wait_blocking(uart1);
        while(uart_is_writable(uart1) && tud_cdc_n_connected(USBD_ITF_CDC_UART1) && tud_cdc_n_available(USBD_ITF_CDC_UART1)){
                int32_t buf;
                 buf = tud_cdc_n_read_char(USBD_ITF_CDC_UART1);
                if(buf < 0){
                        break;
                }
                uart_putc_raw(uart1, buf);


                read_buf_uart1[read_buf_write_uart1] = buf;
                read_buf_write_uart1++;
                if(read_buf_write_uart1 >= sizeof(read_buf_uart1)){
                        read_buf_write_uart1 = 0;

                }

                if(!match_found_uart1){
                        for(int32_t i = 0; i < sizeof(read_buf_uart1); i++){
                                if(read_buf_uart1[i] == 0xC0){
                                        int32_t i_orig = i;
                                        int32_t ii = 0;
                                        for(; ii < sizeof(ESP_SYNC); ii++){
                                                if(read_buf_uart1[i] != ESP_SYNC[ii]){
                                                        break;
                                                }else{
                                                        i++;
                                                        if(i >= sizeof(read_buf_uart1)){
                                                                i = 0;
                                                        }
                                                }
                                        }
                                        if(ii == sizeof(ESP_SYNC)-1){
                                                //ESP Sync magic found, enter flash mode
                                                gpio_pull_up(PIN_ESP32_FLASH);
                                                gpio_pull_up(PIN_ESP32_EN);
                                                sleep_ms(1);
                                                gpio_pull_down(PIN_ESP32_EN);
                                                match_found_uart1 = true;
                                                //TODO: We need to flip this back after ESP32 Image
                                                break;
                                        }else{
                                                i = i_orig;
                                        }
                                }
                        }
                }
        }

        mutex_exit(&tumt_usb_mutex);
        return 0;
}

