#ifndef SYSTEM_MANAGEMENT_H
#define SYSTEM_MANAGEMENT_H

#include "main.h"

#include <stdarg.h>

extern UART_HandleTypeDef huart1;


void uart_transmit_string(UART_HandleTypeDef *huart, uint8_t *string);

void Debug_printf(const char *format, ...);

#endif
