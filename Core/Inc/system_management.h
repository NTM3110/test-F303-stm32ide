#ifndef SYSTEM_MANAGEMENT_H
#define SYSTEM_MANAGEMENT_H

#include "main.h"


void uart_transmit_string(UART_HandleTypeDef *huart, uint8_t *string);

#endif