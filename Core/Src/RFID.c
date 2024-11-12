#include "RFID.h"

// Helper function to control the SS pin
static void CR95HF_Select(void) {
    HAL_GPIO_WritePin(CR95HF_SS_PORT, CR95HF_SS_PIN, GPIO_PIN_RESET);
}

static void CR95HF_Deselect(void) {
    HAL_GPIO_WritePin(CR95HF_SS_PORT, CR95HF_SS_PIN, GPIO_PIN_SET);
}

// Initialize the CR95HF
void CR95HF_Init(void) {
    // Reset or configure GPIO pins for CR95HF as needed
    CR95HF_Deselect();
}

// Send a command to the CR95HF and receive a response
uint8_t CR95HF_SendCommand(uint8_t command, uint8_t* data, uint8_t dataLength, uint8_t* response, uint8_t* responseLength) {
    uint8_t buffer[64];
    buffer[0] = command;
    buffer[1] = dataLength;  // Include data length byte

    // Copy data to buffer starting after command and length bytes
    for (uint8_t i = 0; i < dataLength; i++) {
        buffer[i + 2] = data[i];
    }

    CR95HF_Select();
    if (HAL_SPI_Transmit(&CR95HF_SPI, buffer, dataLength + 2, HAL_MAX_DELAY) != HAL_OK) {
        CR95HF_Deselect();
        return 1;  // Transmission failed
    }

    // Wait for response, check IRQ pin
    while (HAL_GPIO_ReadPin(CR95HF_IRQ_PORT, CR95HF_IRQ_PIN) != GPIO_PIN_RESET) {
        // Optional delay here if needed to avoid busy-waiting
    }

    // Read response
    if (HAL_SPI_Receive(&CR95HF_SPI, response, *responseLength, HAL_MAX_DELAY) != HAL_OK) {
        CR95HF_Deselect();
        return 2;  // Reception failed
    }

    CR95HF_Deselect();
    return CR95HF_RESPONSE_SUCCESS;
}

// Read the CR95HF ID
uint8_t CR95HF_ReadID(uint8_t* idBuffer) {
    uint8_t responseLength = 10;
    return CR95HF_SendCommand(CR95HF_CMD_IDN, NULL, 0, idBuffer, &responseLength);
}

// Select the NFC protocol (e.g., ISO14443A)
uint8_t CR95HF_SelectProtocol(uint8_t protocol) {
    uint8_t protocolData[2] = { protocol, 0x00 };  // Protocol, no additional option
    uint8_t response[10];
    uint8_t responseLength = sizeof(response);
    return CR95HF_SendCommand(CR95HF_CMD_PROTOCOL_SELECT, protocolData, 2, response, &responseLength);
}

// Poll for NFC tag in the field
uint8_t CR95HF_PollField(void) {
    uint8_t response[10];
    uint8_t responseLength = sizeof(response);
    return CR95HF_SendCommand(CR95HF_CMD_POLL_FIELD, NULL, 0, response, &responseLength);
}

// Send and receive data
uint8_t CR95HF_SendRecv(uint8_t* sendData, uint8_t sendLength, uint8_t* recvData, uint8_t* recvLength) {
    return CR95HF_SendCommand(CR95HF_CMD_SEND_RECV, sendData, sendLength, recvData, recvLength);
}
