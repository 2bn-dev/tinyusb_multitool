#pragma once

#include <stdint.h>
#include "serial_io.h"
#include "hardware/uart.h"      //pico-sdk
#include "hardware/gpio.h"	//pico-sdk

typedef struct {
	uart_inst_t * uart;
	uint pin_num_uart_tx;
	uint pin_num_uart_rx;
	uint pin_num_esp32_en;
	uint pin_num_esp32_flash;
} loader_pico_config_t;

void loader_port_pico_init(loader_pico_config_t *config);
