#include "main.h"
#include "cmsis_os.h"
#include "RS232-UART1.h"
#include "Controlling_LED.h"
#include "string.h"



typedef struct
{
  volatile uint32_t* buffer;
  uint16_t size;
  volatile uint32_t* tailPtr;
  DMA_HandleTypeDef* dmaHandle;
} RingBufferDmaU8_ADC_TypeDef;

RingBufferDmaU8_ADC_TypeDef ADC2RxDMARing;

uint32_t adc2Sentence[DMA_STACK_SIZE];
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

void StartControllingLED(void const * argument)
{
  /* USER CODE BEGIN StartControllingLED */
  /* Infinite loop */
	RingBufferDmaU8_initADCRx(&ADC2RxDMARing, &hadc2, adc2Sentence, DMA_STACK_SIZE);
	for(;;)
	{
		Blink_LED(GPIOC, GPIO_PIN_9);
		if(adc2Sentence[0] != 0x00){
			sendUint32Array(adc2Sentence, DMA_STACK_SIZE, &huart1);
			setUint32Array(adc2Sentence,DMA_STACK_SIZE, 0x00000000);
			ADC2_ReInitializeRxDMA();
		}
	}
  /* USER CODE END StartControllingLED */
}
