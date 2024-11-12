#ifndef RFID_H
#define RFID_H

#include "main.h"

// Define CR95HF commands
#define CR95HF_CMD_IDN             0x01
#define CR95HF_CMD_PROTOCOL_SELECT 0x02
#define CR95HF_CMD_POLL_FIELD      0x03
#define CR95HF_CMD_SEND_RECV       0x04

// CR95HF Responses
#define CR95HF_RESPONSE_SUCCESS    0x00

extern SPI_HandleTypeDef hspi2;
// Define SPI and GPIO for CR95HF (adjust to match your setup)
#define CR95HF_SPI                 hspi2         // SPI handle
#define CR95HF_SS_PIN              GPIO_PIN_12    // CS/SS pin
#define CR95HF_SS_PORT             GPIOB         // CS/SS port
#define CR95HF_IRQ_PIN             GPIO_PIN_6    // IRQ pin
#define CR95HF_IRQ_PORT            GPIOC         // IRQ port

// Function prototypes
void CR95HF_Init(void);
uint8_t CR95HF_SendCommand(uint8_t command, uint8_t* data, uint8_t dataLength, uint8_t* response, uint8_t* responseLength);
uint8_t CR95HF_ReadID(uint8_t* idBuffer);
uint8_t CR95HF_SelectProtocol(uint8_t protocol);
uint8_t CR95HF_PollField(void);
uint8_t CR95HF_SendRecv(uint8_t* sendData, uint8_t sendLength, uint8_t* recvData, uint8_t* recvLength);

#endif // CR95HF_H
