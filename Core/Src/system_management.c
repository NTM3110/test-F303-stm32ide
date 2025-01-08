#include "system_management.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "spi_flash.h"

uint32_t current_addr_debug = DEBUG_START_ADDRESS;

void uart_transmit_string(UART_HandleTypeDef *huart, uint8_t *string) {
    HAL_UART_Transmit(huart, string, strlen((char *)string), 1000);
}

void Uint32ToHex(uint32_t value, char *output, uint8_t width) {
    for (int i = 0; i < width; i++) {
        uint8_t nibble = (value >> (4 * (width - 1 - i))) & 0xF; // Extract each nibble
        output[i] = (nibble < 10) ? ('0' + nibble) : ('A' + nibble - 10); // Convert to hex char
    }
    output[width] = '\0'; // Null-terminate
}


void Debug_printf(const char *format, ...) {
    uint8_t output_buffer[256]; // Adjust size as needed
    va_list args;

    // Start processing the variadic arguments
    va_start(args, format);

    // Format the string
    vsnprintf((char*)output_buffer, sizeof(output_buffer), format, args);

    if(is_saving_debug && (strstr((char *)output_buffer, "$GNRMC")) ){
    	uart_transmit_string(&huart1, (uint8_t*) "SAVING GNRMC OUTPUT TO FLASH AT 0X7000");
    	W25_Reset();
    	W25_PageProgram(current_addr_debug, output_buffer, 256);
    	current_addr_debug += 256;
    	char addr_out[10];
    	sprintf(addr_out, "%08lx", current_addr_debug);
    	HAL_UART_Transmit(&huart1, (uint8_t*) addr_out, 8, 1000);
    	HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 1, 1000);
    	if(current_addr_debug == 0x8F00){
    		current_addr_debug = 0x7000;
    		W25_Reset();
    		W25_SectorErase(current_addr_debug);

    	}
    }
    // End processing the arguments
    va_end(args);

    // Transmit the formatted string over UART
    uart_transmit_string(&huart1,(uint8_t*) output_buffer);
}
