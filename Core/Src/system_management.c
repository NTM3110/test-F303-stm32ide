#include "system_management.h"
#include "string.h"

void uart_transmit_string(UART_HandleTypeDef *huart, uint8_t *string) {
    HAL_UART_Transmit(huart, string, strlen((char *)string), 1000);
}