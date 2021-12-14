
#define PICO_DEFAULT_UART_BAUD_RATE 115200

#include "pico/stdlib.h"
#include "pico/time.h"
#include "stdio.h"
#include "pico/stdio_uart.h"
#include "tinyusb_multitool.h"
#include "tumt_uart_bridge.h"
#include "tusb.h"



void fatal_error_infinite_loop(){
	printf("\nExecution Stalled\n");
	while(true){
		_DBG("FATAL ERROR STALL!");
		sleep_ms(1000);
	}
}

void fatal_error(const char * detail){
	printf("\n\nFATAL: %s!\n\n\n", detail);
	fatal_error_infinite_loop();
}

void warning(const char * detail){
	printf("\n\nWARNING: %s!\n\n\n", detail);
}

bool until(absolute_time_t timestamp){
	return (get_absolute_time() <= timestamp);
}


void flash_pending_cb(){
	return;
}

int main(){
	//stdio_uart_init();
	//Disable stdio UART, uart0 is used as bridge below, stdio is available on usb serial port via tumt_usb_init()
	tumt_usb_init(flash_pending_cb);
	tumt_uart_bridge_uart0_init(115200);
	tumt_uart_bridge_uart1_init(921600);
	tumt_uart_bridge_pin_init();

	printf("start!\n");

	uint32_t i = 0;
	while(1){
		const absolute_time_t next_tick = delayed_by_ms(get_absolute_time(), 1000);
		_DBG("test %llu", next_tick);
		while(until(next_tick)){
			/*
			 * tumt_periodic_task() is used to do tinyUSB data transfer callbacks.
			 * The more frequently is called the less latency USB transactions will have.
			 * 
			 * But it should be called frequently in an idle period, the default pico-stdio-usb driver calls it in an interrupt every millisecond.
			 *
			 */

			tumt_periodic_task();
			sleep_ms(1);
		}
		i++;
	}
	return 0;
}


