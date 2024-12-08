#include "main.h"
#include "GPS.h"
#include "string.h"
#include "cmsis_os.h"
#include "stdint.h"
#include <stdio.h>
#include "stdlib.h"
#include "math.h"
#include "system_management.h"
#include <time.h>

#include "RTC.h"
#include "spi_flash.h"

uint8_t rmc_str[128]= {0};
RingBufferDmaU8_TypeDef GPSRxDMARing;

uint8_t gpsSentence[GPS_STACK_SIZE];

osMailQId RMC_MailQFLASHId; // Mail queue identifier FLASH

RMCSTRUCT rmc;
#define GMT 		000

int isRMCExist = 0;
int inx = 0;
int hr=0,min=0,day=0,mon=0,yr=0;
int daychange = 0;

int getRMC_time = 0;



void copy_array(uint8_t *des, uint8_t *src, int size){
	for(size_t i = 0 ;i <  size; i++){
		des[i] = src[i];
	}
}

void GPSUART_ReInitializeRxDMA(void)// ham khoi tao lai DMA
{
	HAL_StatusTypeDef ret = HAL_UART_Abort(&huart2);
	if(ret != HAL_OK)
	{
		Error_Handler();			
	}		
	HAL_Delay(50);	//	50 is OK
	//memset(gnssDmaRingBufferMemory, 0x20, sizeof(gnssDmaRingBufferMemory));	// insert buffer with space character	
	RingBufferDmaU8_initUSARTRx(&GPSRxDMARing, &huart2, gpsSentence, GPS_STACK_SIZE);
}


void display_rmc_data(UART_HandleTypeDef *huart) {

    Debug_printf("Time: %02d:%02d:%02d\r\n", rmc.tim.hour, rmc.tim.min, rmc.tim.sec);

    Debug_printf("Date: %02d/%02d/%04d\r\n", rmc.date.Day, rmc.date.Mon, rmc.date.Yr);
	
    Debug_printf("Latitude: %.6f %c\r\n", rmc.lcation.latitude, rmc.lcation.NS);

    Debug_printf("Longitude: %.6f %c\r\n", rmc.lcation.longitude, rmc.lcation.EW);

    Debug_printf("Speed: %.1f knots\r\n", rmc.speed);

    Debug_printf("Course: %.1f\r\n", rmc.course);

    Debug_printf("Validity: %s\r\n", rmc.isValid ? "Valid" : "Invalid");
}

time_t convertToEpoch(int year, int month, int day, int hour, int min, int sec) {
    struct tm timeinfo;

    // Set timeinfo fields
    timeinfo.tm_year = year + 100; // Year since 1900
    timeinfo.tm_mon = month - 1;    // Month (0-11, so subtract 1)
    timeinfo.tm_mday = day;         // Day of the month
    timeinfo.tm_hour = hour;        // Hour (0-23)
    timeinfo.tm_min = min;          // Minute (0-59)
    timeinfo.tm_sec = sec;          // Second (0-59)
    timeinfo.tm_isdst = -1;         // Automatically determine Daylight Saving Time

    // Convert to epoch time (seconds since 1970-01-01 00:00:00 UTC)
    time_t epoch = mktime(&timeinfo);

    return epoch;
}

void parse_rmc(uint8_t *rmc_sentence) {
    int field = 0;
    uint8_t *ptr = rmc_sentence;

    while (*ptr) {
        if (*ptr == ',' || *ptr == '*') {
            *ptr = '\0';

            switch (field) {
                case 1:  // Time: hhmmss
                    rmc.tim.hour = (rmc_sentence[0] - '0') * 10 + (rmc_sentence[1] - '0');
                    rmc.tim.min = (rmc_sentence[2] - '0') * 10 + (rmc_sentence[3] - '0');
                    rmc.tim.sec = (rmc_sentence[4] - '0') * 10 + (rmc_sentence[5] - '0');
                    break;
                case 2:  // Status
                    rmc.isValid = (rmc_sentence[0] == 'A') ? 1 : 0;
                    break;
                case 3:  // Latitude: ddmm.mmmm
                    rmc.lcation.latitude = (atof((char *)rmc_sentence) )/100;
                    int lati_int = (int)floor(rmc.lcation.latitude);
					float lati_float = rmc.lcation.latitude - lati_int;
					lati_float = lati_float/0.6;
					rmc.lcation.latitude = lati_int + lati_float;
                    break;
                case 4:  // N/S
                    rmc.lcation.NS = rmc_sentence[0];
                    break;
                case 5:  // Longitude: dddmm.mmmm
                    rmc.lcation.longitude = (atof((char *)rmc_sentence))/100;
                    int longi_int = (int)floor(rmc.lcation.longitude);
					float longi_float = rmc.lcation.longitude - longi_int;
					longi_float = longi_float/0.6;
					rmc.lcation.longitude = longi_int + longi_float;
                    break;
                case 6:  // E/W
                    rmc.lcation.EW = rmc_sentence[0];
                    break;
                case 7:  // Speed in knots
                    rmc.speed = atof((char *)rmc_sentence);
                    break;
                case 8:  // Course
                    rmc.course = atof((char *)rmc_sentence);
                    break;
                case 9:  // Date: ddmmyy
                    rmc.date.Day = (rmc_sentence[0] - '0') * 10 + (rmc_sentence[1] - '0');
                    rmc.date.Mon = (rmc_sentence[2] - '0') * 10 + (rmc_sentence[3] - '0');
                    rmc.date.Yr = (rmc_sentence[4] - '0') * 10 + (rmc_sentence[5] - '0');
                    break;
            }

            rmc_sentence = ptr + 1;
            field++;
        }
        ptr++;
    }
	if(rmc.isValid == 1)
		rmc.date.epoch = convertToEpoch(rmc.date.Yr, rmc.date.Mon, rmc.date.Day, rmc.tim.hour, rmc.tim.min, rmc.tim.sec);
}


void sendRMCDataToFlash(RMCSTRUCT *rmcData) {
	HAL_UART_Transmit(&huart1, (uint8_t*) "SENDING RMC TO FLASH\n",  strlen("SENDING RMC\n") , HAL_MAX_DELAY);
    RMCSTRUCT *mail = (RMCSTRUCT *)osMailAlloc(RMC_MailQFLASHId, osWaitForever); // Allocate memory for mail
    if (mail != NULL) {
        *mail = *rmcData; // Copy data into allocated memory
        osMailPut(RMC_MailQFLASHId, mail); // Put message in queue
    }
}


void getRMC(){
	int idx = 0;
	getRMC_time++;
	int length = 0;
	for(size_t i = 0; i < GPS_STACK_SIZE; i++){
		if (gpsSentence[i] == '$' && gpsSentence[i+1] == 'G' && gpsSentence[i+2] == 'N' && gpsSentence[i+3] == 'R' && gpsSentence[i+4] == 'M' && gpsSentence[i+5] == 'C'
			&& (GPS_STACK_SIZE -i) > 200 ){
			isRMCExist = 1;
			HAL_UART_Transmit(&huart1, (uint8_t *)"Getting RMC\n", strlen("Getting RMC\n"), 1000);
			while(gpsSentence[i+1] != 0x0A ){
				rmc_str[idx] = gpsSentence[i];
				idx++;
				i++;
			}
			length = idx;
			idx = 0;
			break;
		}
	 }
	for(size_t i = length; i < 128; i++){
		rmc_str[i] = 0;
	}
	if(isRMCExist == 1){
//		parse_rmc(rmc_str);
//		display_rmc_data(&huart1);
//		set_time(rmc.tim.hour, rmc.tim.min, rmc.tim.sec);
//		set_date(rmc.date.Yr, rmc.date.Mon, rmc.date.Day);
		if(rmc.isValid == 1){
			sendRMCDataToFlash(&rmc);
			getRMC_time = 0;
		}
		isRMCExist = 0;
	}
	if(getRMC_time >= 500){
		GPS_DISABLE();
		osDelay(500);
		GPS_ENABLE();
		getRMC_time = 0;
	}
	Debug_printf("Elapsed Time blabla: %d\n", getRMC_time);
	HAL_UART_Transmit(&huart1, rmc_str, 128,1000);
}


void StartGPS(void const * argument)
{
	HAL_UART_Transmit(&huart1,(uint8_t*) "STARTING GPS", strlen("STARTING GPS"), 1000);
	/* USER CODE BEGIN StartGPS */
	RingBufferDmaU8_initUSARTRx(&GPSRxDMARing, &huart2, gpsSentence, GPS_STACK_SIZE);
//	/* Infinite loop */
	rmc.tim.hour = 0;
	rmc.tim.min = 0;
	rmc.tim.sec = 0;
	rmc.lcation.latitude = 20.998022;
	rmc.lcation.longitude = 105.794756;
	rmc.speed = 22.4;
	rmc.course = 30.5;
	rmc.lcation.NS = 'N';
	rmc.lcation.EW = 'E';
	rmc.isValid = 1;
	rmc.date.Day = 0;
	rmc.date.Mon = 0;
	rmc.date.Yr = 0;
	osMailQDef(FLASH_MailQ, 5, RMCSTRUCT);
	RMC_MailQFLASHId = osMailCreate(osMailQ(FLASH_MailQ), NULL);

	memset(gpsSentence, 0x00, GPS_STACK_SIZE);
	while(1)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
		HAL_Delay(1500);
		getRMC();
		//rmc.lcation.latitude -= 0.000001;
//		rmc.tim.sec += 2;
//		rmc.lcation.latitude = route[count].latitude;
//		rmc.lcation.longitude = route[count].longitude;
//		count++;
		HAL_UART_Transmit(&huart1, (uint8_t *)"Getting GPS \n", strlen("Getting GPS \n"), 1000);
		uart_transmit_string(&huart1,(uint8_t*) "\n\n ");
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_Delay(1500);
	}
  /* USER CODE END StartGPS */
}


