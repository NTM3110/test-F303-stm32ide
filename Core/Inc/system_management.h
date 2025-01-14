#ifndef SYSTEM_MANAGEMENT_H
#define SYSTEM_MANAGEMENT_H

#include "main.h"

#include <stdarg.h>

#define DEBUG_START_ADDRESS 0x7000

extern UART_HandleTypeDef huart1;

extern int is_saving_debug;

void Uint32ToHex(uint32_t value, char *output, uint8_t width);
void Delay_ms(uint32_t duration_ms);

#endif
