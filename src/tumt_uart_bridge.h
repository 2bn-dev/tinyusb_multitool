#define PIN_TXD0 0
#define PIN_RXD0 1
#define PIN_TXD1 4
#define PIN_RXD1 5

#define PIN_ESP32_EN 13
#define PIN_ESP32_FLASH 14

#ifndef TUMT_UART_BRIDGE_TASK_INTERVAL_US
#define TUMT_UART_BRIDGE_TASK_INTERVAL_US 1000
#endif

static int64_t tumt_uart_bridge_uart0_timer(__unused alarm_id_t id, __unused void *user_data);
void tumt_uart_bridge_uart0_init(uint32_t baud_rate);

static int64_t tumt_uart_bridge_uart1_timer(__unused alarm_id_t id, __unused void *user_data);
void tumt_uart_bridge_uart1_init(uint32_t baud_rate);

void tumt_uart_bridge_uart0_deinit();
void tumt_uart_bridge_uart1_deinit();
void tumt_uart_bridge_pin_init(void);

int tumt_uart_bridge_uart0_in_out();
int tumt_uart_bridge_uart1_in_out();
