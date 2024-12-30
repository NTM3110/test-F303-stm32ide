#include "RS232-UART1.h"
#include "time.h"
#include "main.h"
#include "cmsis_os.h"

#define GPS_GPIO_Port		GPIOC	
#define GPS_GPIO_Pin		GPIO_PIN_2

#define GPS_STACK_SIZE 		2048

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;

void getGPS();

void parse_rmc(uint8_t *rmc_sentence);



#define GPS_ENABLE()   HAL_GPIO_WritePin(GPS_GPIO_Port, GPS_GPIO_Pin, GPIO_PIN_RESET)
#define GPS_DISABLE()  HAL_GPIO_WritePin(GPS_GPIO_Port, GPS_GPIO_Pin, GPIO_PIN_SET)

void getRMC();

void GPSUART_ReInitializeRxDMA(void);

void StartGPS(void const * argument);
