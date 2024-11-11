#include "main.h"
#include "cmsis_os.h"


void Blink_LED(GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	osDelay(1000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	osDelay(1000);
}

void StartControllingLED(void const * argument)
{
  /* USER CODE BEGIN StartControllingLED */
  /* Infinite loop */
  for(;;)
  {
    Blink_LED(GPIOC, GPIO_PIN_9);
  }
  /* USER CODE END StartControllingLED */
}