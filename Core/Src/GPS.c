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
#define M_PI 3.14159
#endif

// Define Earth's radius in kilometers
#define EARTH_RADIUS_KM 6371.0

// Convert degrees to radians
#define DEG_TO_RAD(deg) ((deg) * M_PI / 180.0)

uint8_t rmc_str[128]= {0};
uint8_t output_buffer[128] = {0};

RingBufferDmaU8_TypeDef GPSRxDMARing;
extern osMailQId RMC_MailQFLASHId;
uint8_t gpsSentence[GPS_STACK_SIZE];


// Mail queue identifier FLASH
RMCSTRUCT rmc = {0};
RMCSTRUCT rmc_saved = {0};

extern osMutexId myMutexHandle;


#define GMT 		000

int isRMCExist = 0;
int inx = 0;
int hr=0,min=0,day=0,mon=0,yr=0;
int daychange = 0;

int getRMC_time = 0;
extern osMutexId myMutexHandle;

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
	osDelay(100);
}

void enableEASYFuntion(void){
	printf("ENABLE EASY FUNCTION IN GPS");
	HAL_UART_Transmit(&huart2, (uint8_t*)"$PMTK869,1,1*35\r\n", strlen("$PMTK869,1,1*35\r\n"), 2000);
	osDelay(1000);

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
//        snprintf()
        printf("Calculated checksum: %02x\n", calculatedChecksum);
        printf("Received checksum: %02x\n", receivedChecksum);

        // Compare the calculated checksum with the received checksum
        return calculatedChecksum == receivedChecksum;
    }
    printf("Checksum mismatch: calculated 0x%02X, received 0x%02X\n",
                         calculatedChecksum, receivedChecksum);
    return 0; // Invalid checksum
}

void display_rmc_data() {

    printf("Time: %02d:%02d:%02d\r\n", rmc.tim.hour, rmc.tim.min, rmc.tim.sec);

    printf("Date: %02d/%02d/20%02d\r\n", rmc.date.Day, rmc.date.Mon, rmc.date.Yr);
	
    printf("Latitude: %.6f %c\r\n", rmc.lcation.latitude, rmc.lcation.NS);

    printf("Longitude: %.6f %c\r\n", rmc.lcation.longitude, rmc.lcation.EW);

    printf("Speed: %.1f knots\r\n", rmc.speed);

    printf("Course: %.1f\r\n", rmc.course);

    printf("Validity: %s\r\n", rmc.isValid ? "Valid" : "Invalid");
}

time_t convertToEpoch(int year, int month, int day, int hour, int min, int sec) {
    struct tm timeinfo = {0};

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
    strcpy((char*)str_cpy,(char*) rmc_sentence);
    str_cpy[sizeof(str_cpy) - 1] = '\0';

    printf("\n");
    printf((char *)rmc_sentence);
    printf("\n");

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
	 RMCSTRUCT *mail = (RMCSTRUCT *)osMailAlloc(RMC_MailQFLASHId, osWaitForever); // Allocate memory for mail
	if (mail != NULL) {
		*mail = *rmcData; // Copy data into allocated memory
		osStatus status = osMailPut(RMC_MailQFLASHId, mail); // Put message in queue
		if (status != osOK) {
			printf("\n\n-------------------------Failed to send message: %d ------------------------\n\n", status);
		}
		else{
			printf("\n\n-------------------------SEND message successfullly at GPS: %d ------------------------\n\n", status);
		}
	}
	else{
		printf("CANNOT MALLOC MAIL");
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
        printf("Warning: Sentence too long, discarding\n");
        tempIndex = 0; // Reset if line too long
    }
    return 0;
}

void Kalman_Init(KalmanFilter* kf, double processNoise, double measurementNoise, double initialEstimate) {
    kf->x = initialEstimate;
    kf->p = 1.0;  // Initial uncertainty
    kf->q = processNoise;
    kf->r = measurementNoise;
    kf->k = 0.0;  // Kalman gain starts at 0
}

double Kalman_Update(KalmanFilter* kf, double measurement) {
    // Prediction step
    kf->p += kf->q;

    // Update step
    kf->k = kf->p / (kf->p + kf->r);  // Compute Kalman gain
    kf->x += kf->k * (measurement - kf->x);  // Update estimate
    kf->p *= (1.0 - kf->k);  // Update uncertainty

    return kf->x;
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
    if (isRMCExist){
		parse_rmc(rmc_str);// Parse the `$GNRMC` sentence
		display_rmc_data();
		get_RTC_time_date(&rmc);

//		if (rmc.isValid &&
//			(rmc_saved.isValid == 0 ||
//			 isWithinThreshold(rmc_saved.lcation.latitude, rmc_saved.lcation.longitude,
//							   rmc.lcation.latitude, rmc.lcation.longitude, 1.0))) {
		if (rmc.isValid){
			printf("\n\n------------ Sending RMC ------------\n\n");
			sendRMCDataToFlash(&rmc);
			getRMC_time = 0;
			rmc_saved = rmc;
		} else if (rmc_saved.isValid) {
			printf("\n\n------------ GPS BUG: Sending latest RMC ------------\n\n");
			get_RTC_time_date(&rmc_saved);
			sendRMCDataToFlash(&rmc_saved);
		} else{
			printf("\n\n------------ DATA FROM GPS MODULE IS NOT VALID YET ------------\n\n");
		}


        // Clear RMC data after processing
        memset(rmc_str, 0x00, sizeof(rmc_str));
        isRMCExist = 0;
    }

    // GPS timeout logic
    if (getRMC_time >= 150 && getRMC_time % 150 == 0) {
        printf("\n\n-------------------  COLD START GPS module -----------------------\n\n");
        coldStart();
    }

    if (getRMC_time >= 500) {
        GPS_DISABLE();
        osDelay(500);
        GPS_ENABLE();
        getRMC_time = 0;
    }

    printf("Elapsed Time: %d\n", getRMC_time);
}

void StartGPS(void const * argument)
{
	printf("\n\n--------------------STARTING GPS ---------------------\n\n");

//	enableEASYFuntion();
	/* USER CODE BEGIN StartGPS */

	/* Infinite loop */

	RingBufferDmaU8_initUSARTRx(&GPSRxDMARing, &huart2, gpsSentence, GPS_STACK_SIZE);
	memset(gpsSentence, 0x00, GPS_STACK_SIZE);
	while(1)
	{
		if (osMutexWait(myMutexHandle, osWaitForever) == osOK){
			printf("\n\n----------------------- Inside GPS ------------------------\n\n");
	//		uint32_t freeStack2 = osThreadGetStackSpace(GPSHandle);
	//		printf("Thread GPS %p is running low on stack: %04ld bytes remaining\n", GPSHandle, freeStack2);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
			osDelay(500);
//			printf("Hello World!!!!\n");
			getRMC();
			osMutexRelease(myMutexHandle);
//		printf("\n------------------------------ GPS SENTENCE ------------------------------\n");
//		printf((char*) gpsSentence);

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
			osDelay(500);
		}
	}

  /* USER CODE END StartGPS */
}


