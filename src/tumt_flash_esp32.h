#pragma once

#include <stdint.h>
#include "serial_io.h"
#include "hardware/uart.h"      //pico-sdk
#include "hardware/gpio.h"	//pico-sdk



#define ESP32_FAMILY_ID 0x1c5f21b0
#define ESP32_PAYLOAD_LENGTH 452
#define ESP32_BLOCK_SIZE 1024

#define ESP32_INIT_BAUDRATE 115200
#define ESP32_HIGH_BAUDRATE 921600

typedef struct {
	uart_inst_t * uart;
	uint pin_num_uart_tx;
	uint pin_num_uart_rx;
	uint pin_num_esp32_en;
	uint pin_num_esp32_flash;
} loader_pico_config_t;

void loader_port_pico_init(loader_pico_config_t *config);
esp_loader_error_t esp32_verify_flash();
esp_loader_error_t esp32_write_block(size_t target_address, void *payload, size_t write_size);
esp_loader_error_t esp32_erase_flash(size_t address, size_t size, size_t block_size);
esp_loader_error_t esp32_start_flash();
