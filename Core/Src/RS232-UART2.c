// #include "main.h"
// #include "RS232-UART2.h"
// #include "RS232-UART1.h"
// #include "cmsis_os.h"
// #include "string.h"
// #include "stdint.h"
// #include <stdio.h>
// #include "stdlib.h"
// #include "string.h"
// #include "math.h"

// int inx = 0;
// uint8_t RMCbuffer[2048];
// RingBufferDmaU8_TypeDef gpsDecodeRxDMARing;

// uint8_t rmcDecode[128];
// RMCSTRUCT *rmc;

// int hr=0,min=0,day=0,mon=0,yr=0;
// int daychange = 0;

// uint8_t message_gps_decode[] = "GPS DEcode\n";

// void gpsDecode_InitializeRxDMA(void)// ham khoi tao lai DMA
// {
// 	HAL_StatusTypeDef ret = HAL_UART_Abort(&huart1);
// 	if(ret != HAL_OK)
// 	{
// 		Error_Handler();			
// 	}		
// 	HAL_Delay(50);	//	50 is OK
// 	//memset(gnssDmaRingBufferMemory, 0x20, sizeof(gnssDmaRingBufferMemory));	// insert buffer with space character	
// 	RingBufferDmaU8_initUSARTRx(&gpsDecodeRxDMARing, &huart1, RMCbuffer, READLOG_BLOCK_BUFFER_LENGHT);
// }



// int GPS_Decode(){
// 	inx = 0;
// 	char buffer[12];
// 	int i = 0;
// 	while (RMCbuffer[inx] != ',') inx++;  // 1st ,
// 	inx++;
// 	while (RMCbuffer[inx] != ',') inx++;  // After time ,
// 	inx++;
// 	if (RMCbuffer[inx] == 'A')  // Here 'A' Indicates the data is valid, and 'V' indicates invalid data
// 	{
// 		rmc->isValid = 1;
// 	}
// 	else
// 	{
// 		rmc->isValid = 0;
// 		return 1;
// 	}
// 	inx++;
// 	inx++;
// 	while (RMCbuffer[inx] != ','){
// 		buffer[i] = RMCbuffer[inx];
// 		i++;
// 		inx++;
// 	}
// 	if (strlen(buffer) < 6) return 2;  // If the buffer length is not appropriate, return error
// 	int16_t num = (atoi(buffer));   // change the buffer to the number. It will only convert upto decimal
// 	int j = 0;
// 	while (buffer[j] != '.') j++;   // Figure out how many digits before the decimal
// 	j++;
// 	int declen = (strlen(buffer))-j;  // calculate the number of digit after decimal
// 	int dec = atoi ((char *) buffer+j);  // conver the decimal part a a separate number
// 	float lat = (num/100.0) + (dec/pow(10, (declen+2)));  // 1234.56789 = 12.3456789
// 	rmc->lcation.latitude = lat;  // save the lattitude data into the strucure
// 	inx++; 
// 	rmc->lcation.NS = RMCbuffer[inx];  // save the N/S into the structure


// /***********************  GET LONGITUDE **********************/
// 	inx++;  // ',' after NS character
// 	inx++;  // Reach the first number in the longitude
// 	memset(buffer, '\0', 12);
// 	i=0;
// 	while (RMCbuffer[inx] != ',') 
// 	{
// 		buffer[i] = RMCbuffer[inx];
// 		i++;
// 		inx++;
// 	}
// 	num = (atoi(buffer));  // change the buffer to the number. It will only convert upto decimal
// 	j = 0;
// 	while (buffer[j] != '.') j++;  // Figure out how many digits before the decimal
// 	j++;
// 	declen = (strlen(buffer))-j;  // calculate the number of digit after decimal
// 	dec = atoi ((char *) buffer+j);  // conver the decimal part a a separate number
// 	lat = (num/100.0) + (dec/pow(10, (declen+2)));  // 1234.56789 = 12.3456789
// 	rmc->lcation.longitude = lat;  // save the longitude data into the strucure
// 	inx++;
// 	rmc->lcation.EW = RMCbuffer[inx];
	
// 	while (RMCbuffer[inx] != ',') inx++;  // after EW ,

// 	// Get Speed
// 	inx++;
// 	i=0;
// 	memset(buffer, '\0', 12);
// 	while (RMCbuffer[inx] != ',')
// 	{
// 		buffer[i] = RMCbuffer[inx];
// 		i++;
// 		inx++;
// 	}

// 	if (strlen (buffer) > 0){          // if the speed have some data
// 		int16_t num = (atoi(buffer));  // convert the data into the number
// 		int j = 0;
// 		while (buffer[j] != '.') j++;   // same as above
// 		j++;
// 		int declen = (strlen(buffer))-j;
// 		int dec = atoi ((char *) buffer+j);
// 		float lat = num + (dec/pow(10, (declen)));
// 		rmc->speed = lat;
// 	}
// 	else rmc->speed = 0;

// 	// Get Course
// 	inx++;
// 	i=0;
// 	memset(buffer, '\0', 12);
// 	while (RMCbuffer[inx] != ',')
// 	{
// 		buffer[i] = RMCbuffer[inx];
// 		i++;
// 		inx++;
// 	}

// 	if (strlen (buffer) > 0){  // if the course have some data
// 		int16_t num = (atoi(buffer));   // convert the course data into the number
// 		int j = 0;
// 		while (buffer[j] != '.') j++;   // same as above
// 		j++;
// 		int declen = (strlen(buffer))-j;
// 		int dec = atoi ((char *) buffer+j);
// 		float lat = num + (dec/pow(10, (declen)));
// 		rmc->course = lat;
// 	}
// 	else
// 		{
// 			rmc->course = 0;
// 		}

// 	// Get Date
// 	inx++;
// 	i=0;
// 	memset(buffer, '\0', 12);
// 	while (RMCbuffer[inx] != ',')
// 	{
// 		buffer[i] = RMCbuffer[inx];
// 		i++;
// 		inx++;
// 	}

// 	// Date in the format 280222
// 	day = atoi(buffer)/10000;  // extract 28
// 	mon = (atoi(buffer)/100)%100;  // extract 02
// 	yr = atoi(buffer)%100;  // extract 22

// 	day = day+daychange;   // correction due to GMT shift

// 	// save the data into the structure
// 	rmc->date.Day = day;
// 	rmc->date.Mon = mon;
// 	rmc->date.Yr = yr;
// 	char long_str[10];
// 	sprintf(long_str, "%.4f", rmc->lcation.longitude);
// 	HAL_UART_Transmit(&huart1, (uint8_t*) "Longtitude: ", strlen("Longtitude: "), 1000);
// 	HAL_UART_Transmit(&huart1, (uint8_t*)long_str, 10,1000); 
// 	HAL_UART_Transmit(&huart1, (uint8_t*) "\n", strlen("\n"), 1000);
// 	char lat_str[10];
// 	sprintf(lat_str, "%.4f", rmc->lcation.latitude);
// 	HAL_UART_Transmit(&huart1, (uint8_t*) "Latitude: ", strlen("Latitude: "), 1000);
// 	HAL_UART_Transmit(&huart1, (uint8_t*)lat_str, 10,1000); 
// 	HAL_UART_Transmit(&huart1, (uint8_t*) "\n", strlen("\n"), 1000);
// 	memset(RMCbuffer, 0x00, 2048);
// 	gpsDecode_InitializeRxDMA();
// 	return 0;

// }

// void startDecodeGPS(){
// 	HAL_UART_Transmit(&huart1, (uint8_t*) "STARTING GPS Decode", strlen("STARTING GPS Decode"), 1000);
//   /* USER CODE BEGIN StartUART1 */
	
// 	RingBufferDmaU8_initUSARTRx(&gpsDecodeRxDMARing, &huart1, RMCbuffer, 2048);
//   /* Infinite loop */
//   for(;;)
//   {
// 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
// 	osDelay(1000);
// 	GPS_Decode();
// 	//W25_Reset();
// 	HAL_UART_Transmit(&huart1, message_gps_decode, sizeof(message_gps_decode), 100);
// 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
// 	osDelay(1000);
	  
//   }

// }