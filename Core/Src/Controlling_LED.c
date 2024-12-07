#include "main.h"
#include "cmsis_os.h"
#include "RS232-UART1.h"
#include "Controlling_LED.h"
#include "string.h"
#include "system_management.h"
#include <stdio.h>
#include <stdlib.h>
#include "GPS.h"
#include "spi_flash.h"

//uint8_t flashBufferMailReceived[128];


typedef struct
{
  volatile uint32_t* buffer;
  uint16_t size;
  volatile uint32_t* tailPtr;
  DMA_HandleTypeDef* dmaHandle;
} RingBufferDmaU8_ADC_TypeDef;

RingBufferDmaU8_ADC_TypeDef ADC2RxDMARing;
uint32_t adc2Sentence[DMA_STACK_SIZE];

RMCSTRUCT rmc_led;


void RingBufferDmaU8_initADCRx(RingBufferDmaU8_ADC_TypeDef* ring, ADC_HandleTypeDef* hadc, uint32_t* buffer, uint16_t size) // cai dat dma
{
  ring->buffer = buffer;
  ring->size = size;
  ring->tailPtr = buffer;
  ring->dmaHandle = hadc->DMA_Handle;
  HAL_ADC_Start_DMA(hadc, buffer, DMA_STACK_SIZE);
}

void ADC2_ReInitializeRxDMA(void){
	HAL_StatusTypeDef ret = HAL_ADC_Stop_DMA(&hadc2);;
	if(ret != HAL_OK)
	{
		Error_Handler();
	}
	osDelay(50);	//	50 is OK
	RingBufferDmaU8_initADCRx(&ADC2RxDMARing, &hadc2, adc2Sentence, DMA_STACK_SIZE);
}

void Blink_LED(GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	osDelay(1000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	osDelay(1000);
}

void sendUint32Array(uint32_t *array, size_t size, UART_HandleTypeDef *huart) {
    // Create a buffer to store the byte representation of the uint32_t array
    uint8_t byteArray[size * 4];  // Each uint32_t is 4 bytes

    // Convert each uint32_t to 4 bytes
    for (size_t i = 0; i < size; ++i) {
        byteArray[i * 4] = (uint8_t)((array[i] >> 24) & 0xFF);  // MSB
        byteArray[i * 4 + 1] = (uint8_t)((array[i] >> 16) & 0xFF);
        byteArray[i * 4 + 2] = (uint8_t)((array[i] >> 8) & 0xFF);
        byteArray[i * 4 + 3] = (uint8_t)(array[i] & 0xFF);  // LSB
    }

    // Send the byte array using UART
    HAL_UART_Transmit(huart, byteArray, size * 4, 1000);
}

void setUint32Array(uint32_t *array, size_t size, uint32_t value) {
    // Fill the array with the value (memset works on byte-by-byte)
    for (size_t i = 0; i < size; ++i) {
        // Set each uint32_t element individually using memset
        memset(&array[i], value, sizeof(uint32_t));  // Set 4 bytes of the uint32_t element
    }
}

void receiveRMCDataWithAddrLED(){
	uint8_t output_buffer[70];
	uart_transmit_string(&huart1, (uint8_t*)"\\Inside Receiving Data at GSM\n\n");
	osEvent evt = osMailGet(RMC_MailQLEDId, 5000); // Wait for mail
	if(evt.status == osEventMail){
		uart_transmit_string(&huart1, (uint8_t*)"\n\nReceived  ADDRESS Data at GSM: \n");
		GSM_MAIL_STRUCT *receivedData = (GSM_MAIL_STRUCT *)evt.value.p;
		char addr_out[10];
		uart_transmit_string(&huart1, (uint8_t*)"Address received: \n");
		sprintf(addr_out, "%08x\n\n", receivedData->address);
		HAL_UART_Transmit(&huart1, (uint8_t*) addr_out, 8, 1000);
		HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 1, 1000);

		Debug_printf("Location SENDING TO SERVER : %.6f %c, %.6f %c\n", receivedData->rmc.lcation.latitude, receivedData->rmc.lcation.NS, receivedData->rmc.lcation.longitude, receivedData->rmc.lcation.EW);

		Debug_printf("Speed SENDING TO SERVER: %.2f, Course: %.2f, Valid: %d\n", receivedData->rmc.speed, receivedData->rmc.course, receivedData->rmc.isValid);
		osMailFree(RMC_MailQLEDId, receivedData);
	}
}
//void ReceiveUInt8ArrayFromMailQueue(void) {
//    osEvent evt = osMailGet(uint8MailQueue, osWaitForever); // Wait until mail arrives
//    if (evt.status == osEventMail) {
//        UInt8Mail *receivedMail = (UInt8Mail *)evt.value.p;
//
//        // Access the array
//        uint8_t *receivedData = receivedMail->data;
//        size_t length = receivedMail->length;
//
//        // Process the data
//        Debug_printf("Received uint8_t array of length: %d\n", length);
//        for (size_t i = 0; i < length; i++) {
//            Debug_printf("%u",receivedData[i]);
//        }
//
//        // Free allocated memory
//        free(receivedMail->data);           // Free the array memory
//        osMailFree(uint8MailQueue, receivedMail); // Free the mail memory
//    }
//}

void StartControllingLED(void const * argument)
{
	uart_transmit_string(&huart1,(uint8_t*)"\nINSIDE CONTROLLING LED \n");
  /* USER CODE BEGIN StartControllingLED */
  /* Infinite loop */
	//RingBufferDmaU8_initADCRx(&ADC2RxDMARing, &hadc2, adc2Sentence, DMA_STACK_SIZE);

	osMailQDef(RMC_MailLEDQ, 11, GSM_MAIL_STRUCT);
	RMC_MailQLEDId = osMailCreate(osMailQ(RMC_MailLEDQ),NULL);

	for(;;)
	{
		osDelay(1500);
//		Blink_LED(GPIOC, GPIO_PIN_9);
//		if(adc2Sentence[0] != 0x00){
//			sendUint32Array(adc2Sentence, DMA_STACK_SIZE, &huart1);
//			setUint32Array(adc2Sentence,DMA_STACK_SIZE, 0x00000000);
//			ADC2_ReInitializeRxDMA();
//		}
		receiveRMCDataWithAddrGSM();
		osDelay(1500);
	}
  /* USER CODE END StartControllingLED */
}
