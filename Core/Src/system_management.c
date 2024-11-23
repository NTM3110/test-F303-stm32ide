#include "system_management.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

void uart_transmit_string(UART_HandleTypeDef *huart, uint8_t *string) {
    HAL_UART_Transmit(huart, string, strlen((char *)string), 1000);
}


void Debug_printf(const char *format, ...) {
    char output_buffer[256]; // Adjust size as needed
    va_list args;

    // Start processing the variadic arguments
    va_start(args, format);

    // Format the string
    vsnprintf(output_buffer, sizeof(output_buffer), format, args);

    // End processing the arguments
    va_end(args);

    // Transmit the formatted string over UART
    uart_transmit_string(&huart1,(uint8_t*) output_buffer);
}
