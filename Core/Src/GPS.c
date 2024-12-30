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

// Define M_PI if it's not already defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Define Earth's radius in kilometers
#define EARTH_RADIUS_KM 6371.0

// Convert degrees to radians
#define DEG_TO_RAD(deg) ((deg) * M_PI / 180.0)

uint8_t rmc_str[128]= {0};
RingBufferDmaU8_TypeDef GPSRxDMARing;
extern osMessageQueueId_t RMC_MailQFLASHId;
uint8_t gpsSentence[GPS_STACK_SIZE];
// Mail queue identifier FLASH

RMCSTRUCT rmc;
RMCSTRUCT rmc_saved;

#define GMT 		000

int isRMCExist = 0;
int inx = 0;
int hr=0,min=0,day=0,mon=0,yr=0;
int daychange = 0;

int getRMC_time = 0;

// Haversine formula to calculate distance between two lat/lon points
double haversine(double lat1, double lon1, double lat2, double lon2) {
    // Convert degrees to radians
    lat1 = DEG_TO_RAD(lat1);
    lon1 = DEG_TO_RAD(lon1);
    lat2 = DEG_TO_RAD(lat2);
    lon2 = DEG_TO_RAD(lon2);

    // Haversine formula
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    double a = sin(dlat / 2) * sin(dlat / 2) +
               cos(lat1) * cos(lat2) * sin(dlon / 2) * sin(dlon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return EARTH_RADIUS_KM * c;  // Distance in kilometers
}

// Function to check if the new position is within 1 km of the last position
int isWithinThreshold(double lat1, double lon1, double lat2, double lon2, double threshold) {
    double distance = haversine(lat1, lon1, lat2, lon2);
    return distance <= threshold;
}

void copy_array(uint8_t *des, uint8_t *src, int size){
	for(size_t i = 0 ;i <  size; i++){
		des[i] = src[i];
	}
}

void copy_RMC(RMCSTRUCT *rmc_src, RMCSTRUCT *rmc_dest){
	rmc_src->tim.hour = rmc_dest->tim.hour;
	rmc_src->tim.min = rmc_dest->tim.min;
	rmc_src->tim.sec = rmc_dest->tim.sec;
	rmc_src->date.Day = rmc_dest->date.Day;
	rmc_src->date.Mon = rmc_dest->date.Mon;
	rmc_src->date.Yr = rmc_dest->date.Yr;
	rmc_src->lcation.latitude = rmc_dest->lcation.latitude;
	rmc_src->lcation.longitude = rmc_dest->lcation.longitude;
	rmc_src->lcation.NS = rmc_dest->lcation.NS;
	rmc_src->lcation.EW = rmc_dest->lcation.EW;
	rmc_src->speed = rmc_dest->speed;
	rmc_src->course = rmc_dest->course;
	rmc_src->isValid = rmc_dest->isValid;
}

void GPSUART_ReInitializeRxDMA(void)// ham khoi tao lai DMA
{
	HAL_StatusTypeDef ret = HAL_UART_Abort(&huart2);
	if(ret != HAL_OK)
	{
		Error_Handler();			
	}		
	osDelay(50);	//	50 is OK
	//memset(gnssDmaRingBufferMemory, 0x20, sizeof(gnssDmaRingBufferMemory));	// insert buffer with space character	
	RingBufferDmaU8_initUSARTRx(&GPSRxDMARing, &huart2, gpsSentence, GPS_STACK_SIZE);
}

void coldStart(void){
	HAL_UART_Transmit(&huart2, (uint8_t*)"$PMTK104*37\r\n", strlen("$PMTK104*37\r\n"), 2000);
}

// Function to validate the checksum of an NMEA sentence
int validateChecksum(uint8_t *nmeaSentence, size_t len) {
    const uint8_t *start = nmeaSentence;  // Start of the sentence (after '$')
    const uint8_t *checksumStart = NULL;

    // Find the checksum part (after '*')
    for (size_t i = 0; i < len; i++) {
        if (nmeaSentence[i] == '*') {
            checksumStart = &nmeaSentence[i];
            break;
        }
    }

    if (!checksumStart) {
        return 0;  // Invalid sentence format
    }

    uint8_t calculatedChecksum = 0;

    // XOR all characters between '$' and '*', excluding both symbols
    for (const uint8_t *p = start + 1; p < checksumStart; ++p) {
        calculatedChecksum ^= *p;
    }

    // Extract the received checksum (after '*')
    if (checksumStart + 2 < nmeaSentence + len) {
        uint8_t receivedChecksum = (uint8_t)strtol((char *)(checksumStart + 1), NULL, 16);

        // Debugging: Print calculated and received checksums
        Debug_printf("Calculated checksum: %02x\n", calculatedChecksum);
        Debug_printf("Received checksum: %02x\n", receivedChecksum);

        // Compare the calculated checksum with the received checksum
        return calculatedChecksum == receivedChecksum;
    }

    return 0; // Invalid checksum
}

void display_rmc_data(UART_HandleTypeDef *huart) {

    Debug_printf("Time: %02d:%02d:%02d\r\n", rmc.tim.hour, rmc.tim.min, rmc.tim.sec);

    Debug_printf("Date: %02d/%02d/20%02d\r\n", rmc.date.Day, rmc.date.Mon, rmc.date.Yr);
	
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
//    uint8_t str_cpy[128];
//    strcpy(str_cpy, rmc_sentence);
    HAL_UART_Transmit(&huart1, rmc_sentence, 128,1000);
	HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n",1, 1000);

	if(validateChecksum(rmc_sentence, 128) == 0){
		return;
	}

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
					longi_float = longi_float / 0.6;
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
	osStatus_t status = osMessageQueuePut(RMC_MailQFLASHId, rmcData, 0, 1000);
	if (status != osOK) {
	   Debug_printf("\n\n-------------------------Failed to send message: %d ------------------------\n\n", status);
	}
	else{
		Debug_printf("\n\n-------------------------SEND message successfullly at GPS: %d ------------------------\n\n", status);

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
		parse_rmc(rmc_str);
		display_rmc_data(&huart1);
//		set_time(rmc.tim.hour, rmc.tim.min, rmc.tim.sec);
//		set_date(rmc.date.Yr, rmc.date.Mon, rmc.date.Day);
		get_RTC_time_date(&rmc);
		if(rmc.isValid == 1 &&
		  (rmc_saved.isValid == 0 || isWithinThreshold(rmc_saved.lcation.latitude, rmc_saved.lcation.longitude, rmc.lcation.latitude, rmc.lcation.longitude, 1.0)) &&
		  rmc.course > 3){
			Debug_printf("\n\n------------ Sending RMC at GPS------------\n\n");
			sendRMCDataToFlash(&rmc);
			getRMC_time = 0;
			copy_RMC(&rmc_saved, &rmc);
		}
		else{
			if(rmc_saved.isValid == 1){
				Debug_printf("\n\n------------ GPS BUG: Sending latest RMC at GPS------------\n\n");
				get_RTC_time_date(&rmc_saved);
				sendRMCDataToFlash(&rmc_saved);
			}
		}

		isRMCExist = 0;
	}
	else{
		Debug_printf("\n\n------------ GPS MODULE BUG: NO RMC FOUND ------------\n\n");
	}

//	if(getRMC_time >= 150 && getRMC_time % 150 == 0){
//		Debug_printf("\n\n-------------------  COLD START GPS module -----------------------\n\n");
//		coldStart();
//	}
	if(getRMC_time >= 500){
		GPS_DISABLE();
		osDelay(500);
		GPS_ENABLE();
		getRMC_time = 0;
	}
	Debug_printf("Elapsed Time blabla: %d\n", getRMC_time);
//	HAL_UART_Transmit(&huart1, rmc_str, 128,1000);
//	HAL_UART_Transmit(&huart1, (uint8_t*)"\n",1, 1000);
}


void StartGPS(void const * argument)
{
	Debug_printf("\n\n--------------------STARTING GPS ---------------------\n\n");
	/* USER CODE BEGIN StartGPS */

//	/* Infinite loop */
//	rmc.tim.hour = 0;
//	rmc.tim.min = 0;
//	rmc.tim.sec = 0;
//	rmc.lcation.latitude = 20.998022;
//	rmc.lcation.longitude = 105.794756;
//	rmc.speed = 22.4;
//	rmc.course = 30.5;
//	rmc.lcation.NS = 'N';
//	rmc.lcation.EW = 'E';
//	rmc.isValid = 1;
//	rmc.date.Day = 0;
//	rmc.date.Mon = 0;
//	rmc.date.Yr = 0;

	RingBufferDmaU8_initUSARTRx(&GPSRxDMARing, &huart2, gpsSentence, GPS_STACK_SIZE);
	memset(gpsSentence, 0x00, GPS_STACK_SIZE);
	while(1)
	{
//		osThreadId_t thread1 = osThreadGetId();
//		uint32_t freeStack1 = osThreadGetStackSpace(thread1);
//
//		Debug_printf("Thread GPS %p is running low on stack: %04d bytes remaining\n", thread1, freeStack1);
		Debug_printf("\n\n----------------------- Inside GPS ------------------------\n\n");
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
		osDelay(500);
		getRMC();
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		osDelay(500);
	}
  /* USER CODE END StartGPS */
}


