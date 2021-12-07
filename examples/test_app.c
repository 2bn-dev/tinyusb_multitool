
#define PICO_DEFAULT_UART_BAUD_RATE 115200

#include "pico/stdlib.h"
#include "pico/time.h"
#include "stdio.h"
#include "pico/stdio_uart.h"
#include "tinyusb_multitool.h"
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

bool __no_inline_not_in_flash_func(until)(absolute_time_t timestamp){
	return (get_absolute_time() <= timestamp);
}



int __no_inline_not_in_flash_func(main)(){
	stdio_uart_init();
	printf("start!\n");
	tumt_uart_usb_init();
	uint32_t i = 0;
	while(1){
		const absolute_time_t next_tick = delayed_by_ms(get_absolute_time(), 1000);
		_DBG("test2 %llu", next_tick);
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

