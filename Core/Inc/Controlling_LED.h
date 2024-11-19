#ifndef CONTROLLING_LED_H
#define CONTROLLING_LED_H
#include "main.h"

#define DMA_STACK_SIZE 				128

extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

void Blink_LED(GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN);


#endif

