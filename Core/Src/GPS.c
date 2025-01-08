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
extern osMessageQueueId_t RMC_MailQFLASHIdHandle;
uint8_t gpsSentence[GPS_STACK_SIZE];
int count_send_gps = 0;
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
    uint8_t receivedChecksum = 1;

    // XOR all characters between '$' and '*', excluding both symbols
    for (const uint8_t *p = start + 1; p < checksumStart; ++p) {
        calculatedChecksum ^= *p;
    }

    // Extract the received checksum (after '*')
    if (checksumStart + 2 < nmeaSentence + len) {
        receivedChecksum = (uint8_t)strtol((char *)(checksumStart + 1), NULL, 16);

        // Debugging: Print calculated and received checksums
        Debug_printf("Calculated checksum: %02x\n", calculatedChecksum);
        Debug_printf("Received checksum: %02x\n", receivedChecksum);

        // Compare the calculated checksum with the received checksum
        return calculatedChecksum == receivedChecksum;
    }
    Debug_printf("Checksum mismatch: calculated 0x%02X, received 0x%02X\n",
                         calculatedChecksum, receivedChecksum);
    return 0; // Invalid checksum
}

void display_rmc_data() {

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
    uint8_t str_cpy[128];
    strcpy(str_cpy, rmc_sentence);
    str_cpy[sizeof(str_cpy) - 1] = '\0';

    Debug_printf("\n");
    Debug_printf((char *)rmc_sentence);
    Debug_printf("\n");

	if(validateChecksum(rmc_sentence, 128) == 0){
		return;
	}

    uint8_t *ptr = str_cpy;

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
	osStatus_t status = osMessageQueuePut(RMC_MailQFLASHIdHandle, rmcData, 0, 1000);
	if (status != osOK) {
	   Debug_printf("\n\n-------------------------Failed to send message: %d ------------------------\n\n", status);
	}
	else{
		Debug_printf("\n\n-------------------------SEND message successfullly at GPS: %d ------------------------\n\n", status);

	}
}

int handleIncomingChar(char c) {
    static char tempBuffer[256]; // Increased size for safety
    static uint16_t tempIndex = 0;

    if (c == '\n') { // Sentence delimiter
        tempBuffer[tempIndex] = '\0'; // Null-terminate the string
        if (strstr(tempBuffer, "$GNRMC")){ // Detect `$GNRMC`
            strncpy((char*)rmc_str, tempBuffer, sizeof(rmc_str)); // Copy sentence
            tempIndex = 0;
            return 1;
        }
        tempIndex = 0; // Reset for the next sentence
    } else if (tempIndex < sizeof(tempBuffer) - 1) {
        tempBuffer[tempIndex++] = c;
    } else {
        Debug_printf("Warning: Sentence too long, discarding\n");
        tempIndex = 0; // Reset if line too long
    }
    return 0;
}

void getRMC() {
    static uint16_t lastReadIndex = 0; // Tracks the last read position in DMA
    uint16_t writeIndex = GPS_STACK_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);

    // Process new data in the buffer
    while (lastReadIndex != writeIndex) {
        char c = gpsSentence[lastReadIndex];

        // Handle the character and try to detect $GNRMC
        if (handleIncomingChar(c)){
            isRMCExist = 1; // `$GNRMC` sentence is ready
        }

        lastReadIndex = (lastReadIndex + 1) % GPS_STACK_SIZE;
    }

    // Process `$GNRMC` sentence if detected
    if (isRMCExist) {
		parse_rmc(rmc_str);// Parse the `$GNRMC` sentence
		display_rmc_data();
		get_RTC_time_date(&rmc);

//		if (rmc.isValid &&
//			(rmc_saved.isValid == 0 ||
//			 isWithinThreshold(rmc_saved.lcation.latitude, rmc_saved.lcation.longitude,
//							   rmc.lcation.latitude, rmc.lcation.longitude, 1.0))) {
		if (rmc.isValid){
			Debug_printf("\n\n------------ Sending RMC ------------\n\n");
			sendRMCDataToFlash(&rmc);
			count_send_gps++;
			getRMC_time = 0;
			copy_RMC(&rmc_saved, &rmc);
		} else if (rmc_saved.isValid) {
			Debug_printf("\n\n------------ GPS BUG: Sending latest RMC ------------\n\n");
			get_RTC_time_date(&rmc_saved);
			sendRMCDataToFlash(&rmc_saved);
			if(rmc_saved.date.Yr >= 24)
				count_send_gps++;
		} else{
			Debug_printf("\n\n------------ DATA FROM GPS MODULE IS NOT VALID YET ------------\n\n");
		}


        // Clear RMC data after processing
        memset(rmc_str, 0x00, sizeof(rmc_str));
        isRMCExist = 0;
    }

    // GPS timeout logic
    if (getRMC_time >= 150 && getRMC_time % 150 == 0) {
        Debug_printf("\n\n-------------------  COLD START GPS module -----------------------\n\n");
        coldStart();
    }

    if (getRMC_time >= 500) {
        GPS_DISABLE();
        osDelay(500);
        GPS_ENABLE();
        getRMC_time = 0;
    }

    Debug_printf("Elapsed Time: %d\n", getRMC_time);
}
//void getRMC(){
////	int idx = 0;
//	getRMC_time++;
////	int length = 0;
////	for(size_t i = 0; i < GPS_STACK_SIZE; i++){
////		if (gpsSentence[i] == '$' && gpsSentence[i+1] == 'G' && gpsSentence[i+2] == 'N' && gpsSentence[i+3] == 'R' && gpsSentence[i+4] == 'M' && gpsSentence[i+5] == 'C'
////			&& (GPS_STACK_SIZE -i) > 200 ){
////			isRMCExist = 1;
////			HAL_UART_Transmit(&huart1, (uint8_t *)"Getting RMC\n", strlen("Getting RMC\n"), 1000);
////			while(gpsSentence[i+1] != 0x0A ){
////				rmc_str[idx] = gpsSentence[i];
////				idx++;
////				i++;
////			}
////			length = idx;
////			idx = 0;
////			break;
////		}
////	 }
////	for(size_t i = length; i < 128; i++){
////		rmc_str[i] = 0;
////	}
//	ProcessDMAData();
//	if(isRMCExist == 1){
//		parse_rmc(rmc_str);
//		display_rmc_data(&huart1);
////		set_time(rmc.tim.hour, rmc.tim.min, rmc.tim.sec);
////		set_date(rmc.date.Yr, rmc.date.Mon, rmc.date.Day);
//		get_RTC_time_date(&rmc);
//
//		if(rmc.isValid == 1 &&
//		  (rmc_saved.isValid == 0 || isWithinThreshold(rmc_saved.lcation.latitude, rmc_saved.lcation.longitude, rmc.lcation.latitude, rmc.lcation.longitude, 1.0))
//		   ){
//			Debug_printf("\n\n------------ Sending RMC at GPS------------\n\n");
//			sendRMCDataToFlash(&rmc);
//			if(rmc.date.Yr >= 24)
//				count_send_gps++;
//			getRMC_time = 0;
//			copy_RMC(&rmc_saved, &rmc);
//		}
//		else{
//			if(rmc_saved.isValid == 1){
//				Debug_printf("\n\n------------ GPS BUG: Sending latest RMC at GPS------------\n\n");
//				get_RTC_time_date(&rmc_saved);
//				sendRMCDataToFlash(&rmc_saved);
//				count_send_gps++;
//			}
//			else{
//				Debug_printf("\n\n--------------------------------- DATA FROM GPS MODULE IS NOT VALID YET -----------------------------\n\n");
//			}
//		}
//
////		GPSUART_ReInitializeRxDMA();
//		memset(gpsSentence, 0x00, GPS_STACK_SIZE);
//		isRMCExist = 0;
//	}
//	else{
//		Debug_printf("\n\n------------ GPS MODULE BUG: NO RMC FOUND ------------\n\n");
//	}
//
//	if(getRMC_time >= 150 && getRMC_time % 150 == 0){
//		Debug_printf("\n\n-------------------  COLD START GPS module -----------------------\n\n");
//		coldStart();
//	}
//	if(getRMC_time >= 500){
//		GPS_DISABLE();
//		osDelay(500);
//		GPS_ENABLE();
//		getRMC_time = 0;
//	}
//	Debug_printf("Elapsed Time blabla: %d\n", getRMC_time);
////	HAL_UART_Transmit(&huart1, rmc_str, 128,1000);
////	HAL_UART_Transmit(&huart1, (uint8_t*)"\n",1, 1000);
//}


void StartGPS(void const * argument)
{
	Debug_printf("\n\n--------------------STARTING GPS ---------------------\n\n");
	/* USER CODE BEGIN StartGPS */

	/* Infinite loop */

	rmc_saved = readFlash(0x9000);
	Debug_printf("\n-------------------------- BACK UP GPS FROM FLASH ----------------------- \n");
	if(rmc_saved.isValid == 0){
		Debug_printf("There is not back up GPS from FLASH");
		rmc_saved.tim.hour = 0;
		rmc_saved.tim.min = 0;
		rmc_saved.tim.sec = 0;
		rmc_saved.lcation.latitude = 20.998022;
		rmc_saved.lcation.longitude = 105.794756;
		rmc_saved.speed = 22.4;
		rmc_saved.course = 30.5;
		rmc_saved.lcation.NS = 'N';
		rmc_saved.lcation.EW = 'E';
		rmc_saved.isValid = 1;
		rmc_saved.date.Day = 0;
		rmc_saved.date.Mon = 0;
		rmc_saved.date.Yr = 0;
	}

	RingBufferDmaU8_initUSARTRx(&GPSRxDMARing, &huart2, gpsSentence, GPS_STACK_SIZE);
	memset(gpsSentence, 0x00, GPS_STACK_SIZE);
	while(1)
	{
		Debug_printf("\n\n----------------------- Inside GPS ------------------------\n\n");
		uint32_t freeStack2 = osThreadGetStackSpace(GPSHandle);
		Debug_printf("Thread GPS %p is running low on stack: %04d bytes remaining\n", GPSHandle, freeStack2);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
		osDelay(500);
		getRMC();
		Debug_printf("\n\n ---------------------------------------------- COUNT SEND GPS: %d ---------------------------------------- \n\n", count_send_gps);
		if(count_send_gps == 29)
			count_send_gps = 0;
//		Debug_printf("\n------------------------------ GPS SENTENCE ------------------------------\n");
//		Debug_printf((char*) gpsSentence);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		osDelay(500);
	}

  /* USER CODE END StartGPS */
}


