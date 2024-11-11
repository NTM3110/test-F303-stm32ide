#include "RS232-UART1.h"
#include "time.h"

#define GPS_GPIO_Port		GPIOC	
#define GPS_GPIO_Pin		GPIO_PIN_2

#define GPS_STACK_SIZE 		2048
typedef struct {
	int hour;
	int min;
	int sec;
}TIME;

typedef struct {
	double latitude;
	char NS;
	double longitude;
	char EW;
}LOCATION;

typedef struct {
	int Day;
	int Mon;
	int Yr;
	time_t epoch;
}DATE;

typedef struct {
	TIME tim;
	DATE date;
	float speed;
	float course;
	int isValid;
	LOCATION lcation;
}RMCSTRUCT;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;

void getGPS();
void GPS_Decode(RMCSTRUCT* rmc);

void parse_rmc(uint8_t *rmc_sentence);


#define GPS_ENABLE()   HAL_GPIO_WritePin(GPS_GPIO_Port, GPS_GPIO_Pin, GPIO_PIN_RESET)
#define GPS_DISABLE()  HAL_GPIO_WritePin(GPS_GPIO_Port, GPS_GPIO_Pin, GPIO_PIN_SET)

void getRMC();

void GPSUART_ReInitializeRxDMA(void);

void StartGPS(void const * argument);