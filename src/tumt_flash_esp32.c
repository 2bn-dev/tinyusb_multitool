#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include "tumt_flash_esp32.h"
#include "serial_io.h"  	//esp-serial-flasher
#include "esp_loader.h" 	//esp-serial-flasher
#include "pico/time.h"  	//pico-sdk


static absolute_time_t s_time_end;
static uint pin_num_uart_tx, pin_num_uart_rx;
static uint pin_num_esp32_en, pin_num_esp32_flash;
static uart_inst_t * uart;


//Pins for ESP32_EN and ESP32_FLASH are on an inverter on 2bn LED matrix board
void loader_port_enter_bootloader(void){
	gpio_pull_up(pin_num_esp32_flash);
	loader_port_reset_target();
	loader_port_delay_ms(50);
	gpio_pull_down(pin_num_esp32_flash);
}

void loader_port_reset_target(void){
	gpio_pull_up(pin_num_esp32_en);
	loader_port_delay_ms(200);
	gpio_pull_down(pin_num_esp32_en);
}

void loader_port_delay_ms(uint32_t ms){
	busy_wait_ms(ms);
}

#ifdef SERIAL_DEBUG_ENABLE

static void dec_to_hex_str(const uint8_t dec, uint8_t hex_str[3]){
	static const uint8_t dec_to_hex[] = {
		'0', '1', '2', '3', '4', '5', '6', '7',
		'8', '9', 'a', 'b', 'c', 'd', 'e', 'f'
	};

	hex_str[0] = dec_to_hex[(dec >> 4)];
	hex_str[1] = dec_to_hex[(dec & 0xF)];
	ex_str[2] = '\0';
}

static void serial_debug_print(const uint8_t *data, uint16_t size, bool write){
	static bool write_prev = false;
	uint8_t hex_str[3];

	if(write_prev != write) {
		write_prev = write;
		printf("\n--- %s ---\n", write ? "WRITE" : "READ");
	}

	for(uint32_t i = 0; i < size; i++) {
		dec_to_hex_str(data[i], hex_str);
		printf("%s ", hex_str);
	}
}

#else

static void serial_debug_print(const uint8_t *data, uint16_t size, bool write) { }

#endif


esp_loader_error_t connect_to_target(uint32_t higher_baudrate){
    esp_loader_connect_args_t connect_config = ESP_LOADER_CONNECT_DEFAULT();

    esp_loader_error_t err = esp_loader_connect(&connect_config);
    if (err != ESP_LOADER_SUCCESS) {
        printf("Cannot connect to target. Error: %u\n", err);
        return err;
    }
    printf("Connected to target\n");

    if (higher_baudrate && esp_loader_get_target() != ESP8266_CHIP) {
        err = esp_loader_change_baudrate(higher_baudrate);
        if (err == ESP_LOADER_ERROR_UNSUPPORTED_FUNC) {
            printf("ESP8266 does not support change baudrate command.");
            return err;
        } else if (err != ESP_LOADER_SUCCESS) {
            printf("Unable to change baud rate on target.");
            return err;
        } else {
            err = loader_port_change_baudrate(higher_baudrate);
            if (err != ESP_LOADER_SUCCESS) {
                printf("Unable to change baud rate.");
                return err;
            }
            printf("Baudrate changed\n");
        }
    }


    return ESP_LOADER_SUCCESS;
}

esp_loader_error_t loader_port_serial_write(const uint8_t *data, uint16_t size, uint32_t timeout){
	serial_debug_print(data, size, true);

	//timeout is ignored :/
	uart_write_blocking(uart, data, size);
	uart_tx_wait_blocking(uart);

	return ESP_LOADER_SUCCESS;
	//return ESP_LOADER_ERROR_TIMEOUT;
	//return ESP_LOADER_ERROR_FAIL;
}
esp_loader_error_t loader_port_serial_read(uint8_t *data, uint16_t size, uint32_t timeout){
	memset(data, 0x0, size);

	if(!uart_is_readable_within_us(uart, timeout*1000)){
		return ESP_LOADER_ERROR_TIMEOUT;
	}

	uart_read_blocking(uart, data, (size_t) size);
	

	serial_debug_print(data, size, false);

	return ESP_LOADER_SUCCESS;
	//return ESP_LOADER_ERROR_FAIL;
}

void loader_port_pico_init(loader_pico_config_t *config){
	uart 			= config->uart;
	pin_num_uart_tx 	= config->pin_num_uart_tx;
	pin_num_uart_rx 	= config->pin_num_uart_rx;
	pin_num_esp32_flash 	= config->pin_num_esp32_flash;
	pin_num_esp32_en 	= config->pin_num_esp32_en;

	uart_init(uart, ESP32_INIT_BAUDRATE);
	gpio_set_function(pin_num_uart_tx, GPIO_FUNC_UART);
	gpio_set_function(pin_num_uart_rx, GPIO_FUNC_UART);
}

void loader_port_start_timer(uint32_t ms){
	s_time_end = make_timeout_time_ms(ms);
}


uint32_t loader_port_remaining_time(void){
	int32_t remaining = absolute_time_diff_us(get_absolute_time(), s_time_end)/1000;
	return (remaining > 0) ? (uint32_t)remaining : 0;
}


void loader_port_debug_print(const char *str){
	printf("DEBUG: %s", str);
}

esp_loader_error_t loader_port_change_baudrate(uint32_t baudrate){
	//Change uart baudrate
	if( uart_set_baudrate(uart0, baudrate) != baudrate ) {
		return ESP_LOADER_ERROR_FAIL;
	}

	return ESP_LOADER_SUCCESS;
}

esp_loader_error_t esp32_start_flash(){
	loader_pico_config_t config =  {
	        .uart = uart0,
	        .pin_num_uart_tx = 0,
	        .pin_num_uart_rx = 1,
	        .pin_num_esp32_en = 13,
	        .pin_num_esp32_flash = 14,
	};

	loader_port_pico_init(&config);

	return connect_to_target(ESP32_HIGH_BAUDRATE);
}

esp_loader_error_t esp32_erase_flash(size_t address, size_t size, size_t block_size){
	esp_loader_error_t err;

	printf("Erasing flash (this may take a while)...\n");
	err = esp_loader_flash_start(address, size, block_size);
	if (err != ESP_LOADER_SUCCESS) {
		printf("Erasing flash failed with error %d.\n", err);
		return err;
	}

	return ESP_LOADER_SUCCESS;
}


esp_loader_error_t esp32_write_block(size_t target_address, void *payload, size_t write_size){
	esp_loader_error_t result;

	result = esp_loader_flash_write(payload, write_size);
	if (result != ESP_LOADER_SUCCESS) {
		printf("\nESP32 Block write fail 0x%x - %d.\n", target_address, result);
	}

	return result;
}

esp_loader_error_t esp32_verify_flash(){
#if MD5_ENABLED
	esp_loader_error_t result;

	result = esp_loader_flash_verify();
	if (result == ESP_LOADER_ERROR_UNSUPPORTED_FUNC) {
		printf("ESP8266 does not support flash verify command.");
		return result;
	} else if (result != ESP_LOADER_SUCCESS) {
		printf("MD5 does not match. err: %d\n", result);
		return result;
	}

	printf("Flash VERIFY SUCCESS\n");
#endif

	return ESP_LOADER_SUCCESS;
}
