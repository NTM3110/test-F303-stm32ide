#include "system_management.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "spi_flash.h"

uint32_t current_addr_debug = DEBUG_START_ADDRESS;
uint8_t output_debug_buffer[512] = {0};


void Uint32ToHex(uint32_t value, char *output, uint8_t width) {
    for (int i = 0; i < width; i++) {
        uint8_t nibble = (value >> (4 * (width - 1 - i))) & 0xF; // Extract each nibble
        output[i] = (nibble < 10) ? ('0' + nibble) : ('A' + nibble - 10); // Convert to hex char
    }
    output[width] = '\0'; // Null-terminate
}
void Delay_ms(uint32_t duration_ms) {
    for (uint32_t i = 0; i < duration_ms; i++) {
        for (volatile uint32_t j = 0; j < 666; j++) {
            // Inner loop to create 1 ms delay
        }
    }
}
