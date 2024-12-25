#include "string.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "GPS.h"
#include "GSM.h"
#include "RTC.h"
#include "spi_flash.h"
#include "system_management.h"
#include "Queue_GSM.h"

enum MODE{
	MAIL,
	STORAGE
};

uint8_t TERMINAL_REGISTRATION[128];
uint8_t response[SIM_RESPONSE_MAX_SIZE];
RingBufferDmaU8_TypeDef SIMRxDMARing;
int is_activated = 0;
int is_set_time = 0;
int received_RMC = 0;
int is_in_sending = 0;
int is_keep_up = 0;
int in_getting_mail_stack = 0;
int num_in_mail_sent = 0;
int result_final = 1;

volatile uint32_t start_addr_not_ready = 0;
volatile uint32_t end_addr_not_ready = 0;
volatile uint32_t current_addr_not_ready = 0;

//int is_40s = 0;
RMCSTRUCT rmc_jt;
uint8_t terminal_phone_number[6] = {0};
GSM_MAIL_STRUCT receivedDataGSM;


JT808_TerminalRegistration create_terminal_registration(){
	JT808_TerminalRegistration reg_msg = {
        .start_mask = 0x7E,
        .message_type = {0x01, 0x00},
        .message_length = {0x00, 0x2D},
        .terminal_phone_number = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        .message_serial_number = {0x00, 0x03},
        .province_ID = {0x00, 0x00},
        .city_ID = {0x00, 0x00},
        .manufacturer_ID = {0x00, 0x00, 0x00, 0x00, 0x00},
        .terminal_type = {0x41, 0x35, 0x4D, 0x00, 0x00, 0x00, 0x00, 0x00},
        .terminal_ID = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        .plate_color = 0x00,
        .plate_no = {0x00, 0x00, 0x00, 0x00, 0x35, 0x36, 0x37, 0x38, 0x39, 0x31, 0x20, 0x32, 0x39, 0x4C, 0x31, 0x33, 0x34, 0x31, 0x35, 0x34},
        .check_sum = 0x00,  // Placeholder, will be set by the function
        .end_mask = 0x7E
    };

	return reg_msg;
}


JT808_LocationInfoReport create_location_info_report() {
	JT808_LocationInfoReport location_info = {
        .start_mask = 0x7E,                    // 7E
        .message_type = {0x02, 0x00},          // 02 00
        .message_length = {0x00, 0x32},        // 00 32
        .terminal_phone_number = {0x00, 0x12, 0x34, 0x56, 0x78, 0x91}, // 00 12 34 56 78 91
        .terminal_serial_number = {0x00, 0x0A}, // 00 0A
        .alarm = {0x00, 0x00, 0X00, 0X00},     // 00 00 00 00
        .status = {0x00, 0x00, 0x00, 0x00},    // 00 00 00 00
        .latitude = {0x00, 0x00, 0x00, 0x00},  // 01 40 67 86
        .longitude = {0x00, 0x00, 0x00, 0x00}, // 06 4E 4C C4
        .altitude = {0x00, 0x00},              // 00 00
        .speed = {0x00, 0x00},                 // 00 00
        .direction = {0x00, 0x00},             // 00 00
        .timestamp = {0x24, 0x11, 0x08, 0x17, 0x20, 0x00}, // 24 11 08 17 10 00
        .mileage = {0x01, 0x04, 0x00, 0x00, 0x00, 0x00}, // 01 04 00 00 00 00
        .oil = {0x2A, 0x02},                   // 2A 02
        .driving_record_speed = {0x00, 0x00},  // 00 00
        .vehicle_id = {0x30, 0x01, 0x13},      // 30 01 13
        .signal = {0x31},                      // 31
        .additional = {0x01, 0x00, 0xFD, 0x04, 0x03, 0xF1, 0x00, 0x00, 0x0A}, // 01 00 FD 04 03 F1 00 00 0A
        .end_mask = 0x7E                       // 7E
    };
    
    return location_info;
}


void setBit(uint8_t *status, int bitPosition) {
    *status |= (1 << bitPosition);  // Set the specific bit to 1
}

void clearBit(uint8_t *status, int bitPosition) {
    *status &= ~(1 << bitPosition); // Clear the specific bit to 0
}

void set_status_bit(uint8_t *status_bit){
	if(rmc_jt.lcation.NS == 'N') clearBit(status_bit+3, 2);
	else setBit(status_bit+3, 2);

	if(rmc_jt.lcation.EW == 'E') clearBit(status_bit+3, 3);
	else setBit(status_bit+3, 3);
}

uint8_t calculate_checksum(uint8_t *data, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 1; i < length - 2; i++) {  // Skip start and end markers
        checksum ^= data[i];
    }
    return checksum;
}

uint8_t *convert_location_info_to_array(JT808_LocationInfoReport *location_info, size_t *array_length) {
    *array_length = sizeof(JT808_LocationInfoReport);
    uint8_t *message_array = malloc(*array_length);

    if (message_array == NULL) {
        return NULL;  // Allocation failed
    }

    memcpy(message_array, location_info, *array_length);  // Copy struct data into message array

    return message_array;
}

void send_AT_command(const char *command) {
    HAL_UART_Transmit(&huart3, (uint8_t *)command, strlen(command), HAL_MAX_DELAY);
}

void SIM_UART_ReInitializeRxDMA(void){
	HAL_StatusTypeDef ret = HAL_UART_Abort(&huart3);
	if(ret != HAL_OK)
	{
		Error_Handler();			
	}		
	osDelay(50);	//	50 is OK
	//memset(gnssDmaRingBufferMemory, 0x20, sizeof(gnssDmaRingBufferMemory));	// insert buffer with space character	
	RingBufferDmaU8_initUSARTRx(&SIMRxDMARing, &huart3, response, SIM_RESPONSE_MAX_SIZE);
}

int find_length(uint8_t *str){
	int i = 0;
	while(str[i] != 0x00){
		i++;
	}
	return i;
		
}

void receive_response(char *cmd_str) {
	uint8_t output_buffer[128];
	snprintf((char *)output_buffer, 128, "Response at command: %s\n", cmd_str);
	uart_transmit_string(&huart1, output_buffer);
	//while(response[1] == '\0'){}

	HAL_UART_Transmit(&huart1, response, find_length(response), 1000);
	uart_transmit_string(&huart1, (uint8_t*)"\n");
//	osDelay(1000);
}

void init_SIM_module() {
    
    // Check if module responds
	SIM_ENABLE();
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	osDelay(2000);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
}

void reboot_SIM_module(){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	osDelay(1500);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
	osDelay(10000);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	osDelay(1500);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
}

int convert_dec_to_hex_value(int int_value){
    return (int_value /10*16) + (int_value%10);
}


void save_rmc_to_location_info(JT808_LocationInfoReport* location_info){
	location_info->timestamp[0] = (uint8_t)convert_dec_to_hex_value(rmc_jt.date.Yr);  // Assign year (0x23)
	location_info->timestamp[1] = (uint8_t)convert_dec_to_hex_value(rmc_jt.date.Mon);          // Assign month (0x11)
	location_info->timestamp[2] = (uint8_t)convert_dec_to_hex_value(rmc_jt.date.Day);            // Assign day (0x08)
	location_info->timestamp[3] = (uint8_t)convert_dec_to_hex_value(rmc_jt.tim.hour);           // Assign hour (0x14)
	location_info->timestamp[4] = (uint8_t)convert_dec_to_hex_value(rmc_jt.tim.min);         // Assign minute (0x55)
	location_info->timestamp[5] = (uint8_t)convert_dec_to_hex_value(rmc_jt.tim.sec);  
	double latitude = rmc_jt.lcation.latitude * 1000000;
	double longitude = rmc_jt.lcation.longitude * 1000000;
	int32_t latitude_int = (int32_t)round(latitude);  // Convert to integer, rounding if needed
	int32_t longitude_int = (int32_t)round(longitude);
	
	location_info->latitude[0] = (latitude_int >> 24) & 0xFF;  // Most significant byte
	location_info->latitude[1] = (latitude_int >> 16) & 0xFF;
	location_info->latitude[2] = (latitude_int >> 8) & 0xFF;
	location_info->latitude[3] = latitude_int & 0xFF;
	
	location_info->longitude[0] = (longitude_int >> 24) & 0xFF;  // Most significant byte
	location_info->longitude[1] = (longitude_int >> 16) & 0xFF;
	location_info->longitude[2] = (longitude_int >> 8) & 0xFF;
	location_info->longitude[3] = longitude_int & 0xFF;
	
	int speed = round(rmc_jt.speed * 10 * 1.852);
	location_info->speed[0] = (speed >> 8) & 0xFF;
	location_info->speed[1] =  speed & 0xFF;
	
	int direction = round(rmc_jt.course);
	location_info->direction[0] = (direction >> 8) & 0xFF;
	location_info->direction[1] =  direction & 0xFF;
	
	set_status_bit(location_info->status);
}

//AT
int first_check_SIM()
{
	memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
	SIM_UART_ReInitializeRxDMA();
	const char *substring = "PB DONE";
	int count_check = 0;
	memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
	SIM_UART_ReInitializeRxDMA();
	while(strstr((char *) response, substring) == NULL)
	{
		receive_response("WAITING FOR SIM MODULE TO BE READY\n");
		osDelay(1000);
		if(count_check >= 40) return 0;
		osDelay(200);
	}
	receive_response("WAITING FOR SIM MODULE TO BE READY\n");
	osDelay(1000);
//	int extract_result = extract_time(response);
//	if(extract_result == 0) return 0;

	memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
	SIM_UART_ReInitializeRxDMA();

	send_AT_command(FIRST_CHECK);
	while(strstr((char *) response, CHECK_RESPONSE) != NULL){
		receive_response("First check SIM MODULE\n");
	}
	receive_response("First check SIM MODULE\n");
	memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
	SIM_UART_ReInitializeRxDMA();

	send_AT_command("AT+CPAS\r\n");
	while(strstr((char *) response, CHECK_RESPONSE) != NULL){
		receive_response("Check status of SIM MODULE\n");
		osDelay(1000);
	}
	receive_response("Check status of SIM MODULE\n");
	memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
	SIM_UART_ReInitializeRxDMA();

	send_AT_command("AT+CMEE=2\r\n");
	while(strstr((char *) response, CHECK_RESPONSE) != NULL){
		receive_response("Check enable result code\n");
		osDelay(1000);
	}
	receive_response("Check enable result code\n");
	memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
	SIM_UART_ReInitializeRxDMA();

	return 1;
}


void extract_last_12_digits_bcd(const uint8_t *response, uint8_t *output) {
	uint8_t output_buffer[10];
	const uint8_t *start = response;
	while (*start && !(start[0] == 'A' && start[1] == 'T' && start[2] == '+' &&
					   start[3] == 'C' && start[4] == 'G' && start[5] == 'S' &&
					   start[6] == 'N' && start[7] == '=' && start[8] == '1')) {
		start++;
	}
	uart_transmit_string(&huart1, (uint8_t *)"Inside Checking terminal Number: ");
	uart_transmit_string( &huart1,(uint8_t *) start);
	// If "AT+CGSN=1" is found, move to the start of the number (skip "AT+CGSN=1 ")
	if (*start) {
		start += 10;  // Move pointer past "AT+CGSN=1 "

		// Move past any non-numeric characters
		while (*start && (*start < '0' || *start > '9')) {
			start++;
		}

		// Find the length of numeric sequence and start at the last 12 digits
		const uint8_t *end = start;
		size_t digit_count = 0;
		while (*end && (*end >= '0' && *end <= '9')) {
			end++;
			digit_count++;
		}
		uart_transmit_string(&huart1, (uint8_t *)"Inside Checking terminal Number-2: LEN ");
		snprintf((char*)output_buffer, 10, "%d", digit_count);
		if (digit_count >= 12) {
			const uint8_t *last_12 = end - 12;

			// Convert each pair of digits to BCD
			for (int i = 0; i < 6; i++) {
				output[i] = ((last_12[i * 2] - '0') << 4) | (last_12[i * 2 + 1] - '0');
			}
		}
	}
	uart_transmit_string(&huart1, (uint8_t *)" Check terminal Number-0: ");
	uart_transmit_string(&huart1, output);
	uart_transmit_string(&huart1, (uint8_t *)" \n");
}

int extractCSQValues(const char *input, int *rssi, int *ber) {
    char *start = strstr(input, "+CSQ: ");
    if (start) {
        start += 6; // Move past "+CSQ: "
        char *comma = strchr(start, ',');
        if (comma) {
            *comma = '\0'; // Temporarily terminate the first number
            *rssi = atoi(start); // Convert RSSI string to integer
            *ber = atoi(comma + 1); // Convert BER string to integer
            return 0; // Success
        }
    }
    return -1; // Failure
}

int check_SIM_ready(){
	const int TIME_LIMIT = 5;
	int count_check_sim = 0;
	//GET IMEI
	send_AT_command(GET_IMEI);
	while(strstr((char *) response, CHECK_RESPONSE) == NULL){
		receive_response("Check IMEI-0:\n");
		osDelay(1000);
	}
	receive_response("Check IMEI-0:\n");
	extract_last_12_digits_bcd(response, terminal_phone_number);
	uart_transmit_string(&huart1, (uint8_t *)" Check terminal Number: ");
	uart_transmit_string(&huart1, terminal_phone_number);
	uart_transmit_string(&huart1, (uint8_t *)" \n");
	osDelay(100);
	memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
	SIM_UART_ReInitializeRxDMA();

	//GET MODEL IDENTIFICATION
	send_AT_command(GET_MODEL_IDENTI);
	while(strstr((char *) response, CHECK_RESPONSE) == NULL){
		receive_response("Check MODEL IDENTIFICATION\n");
		osDelay(1000);
	}
	receive_response("Check MODEL IDENTIFICATION\n");
	osDelay(100);
	memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
	SIM_UART_ReInitializeRxDMA();


	// Check if SIM is ready
	send_AT_command(CHECK_SIM_READY);
	osDelay(100);
	while(strstr((char *) response, CHECK_RESPONSE) == NULL){
		receive_response("Check SIM\n");
		osDelay(1000);
		count_check_sim++;
		if (count_check_sim >= 5){
			memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
			SIM_UART_ReInitializeRxDMA();
			return 0;
		}
	}
	receive_response("Check SIM\n");
	osDelay(100);
	memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
	SIM_UART_ReInitializeRxDMA();
	osDelay(100);
	count_check_sim = 0;


	//GET SIM CCID
	send_AT_command(GET_SIM_CCID);
	while(strstr((char *) response, "+QCCID:") == NULL){
		receive_response("Check SIM CCID\n");
		osDelay(1000);
		count_check_sim++;
		if (count_check_sim >= TIME_LIMIT){
			memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
			SIM_UART_ReInitializeRxDMA();
			return 0;
		}
	}
	receive_response("Check SIM CCID\n");
	osDelay(100);
	memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
	SIM_UART_ReInitializeRxDMA();
	count_check_sim = 0;


	// Configuring Network Registration Status (CS Service)
	send_AT_command(CONFIGURE_CS_SERVICE);
	char *first_pointer = NULL;
	char *second_pointer = NULL;
	receive_response("Configuring Network Registration Status (CS Service)");
	while (first_pointer == NULL || second_pointer == NULL){
		send_AT_command("AT+CREG?\r\n");
		osDelay(150);
		receive_response("Check Network Registration Status (CS Service)\n");
		osDelay(1000);
		osDelay(300);
		first_pointer = strstr((char*)response, CHECK_RESPONSE);
		if(first_pointer != NULL){
			second_pointer = strstr(first_pointer+1, CHECK_RESPONSE);
		}
		if (count_check_sim >= TIME_LIMIT){
			memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
			SIM_UART_ReInitializeRxDMA();
			return 0;
		}
	}
	osDelay(100);
	memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
	SIM_UART_ReInitializeRxDMA();
	count_check_sim = 0;


	//Configuring Network Registration Status (PS Service)
	send_AT_command(CONFIGURE_PS_SERVICE);
	first_pointer = NULL;
	second_pointer = NULL;
	receive_response("Configuring Network Registration Status (PS Service)");
	while (first_pointer == NULL || second_pointer == NULL){
		send_AT_command("AT+CGREG?\r\n");
		osDelay(150);
		receive_response("Check Network Registration Status (PS Service)\n");
		osDelay(1000);
		osDelay(300);
		first_pointer = strstr((char*)response, CHECK_RESPONSE);
		if(first_pointer != NULL){
			second_pointer = strstr(first_pointer + 1, CHECK_RESPONSE);
		}
		if (count_check_sim >= TIME_LIMIT){
			memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
			SIM_UART_ReInitializeRxDMA();
			return 0;
		}
	}
	osDelay(100);
	receive_response("Check Network Registration Status (PS Service)\n");
	memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
	SIM_UART_ReInitializeRxDMA();
	count_check_sim = 0;


	//CHECK SIGNAL QUALITY
	send_AT_command(CHECK_SIGNAL_QUALITY);
	while(strstr((char *) response, CHECK_RESPONSE) == NULL){
		receive_response("Check Signal Quality Report\n");
	}
	receive_response("Check Signal Quality Report\n");
	osDelay(100);
	memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
	SIM_UART_ReInitializeRxDMA();
	


	return 1;
}

void check_configure_APN(){
	send_AT_command(CHECK_CONFIGURE_APN);
	osDelay(150);
	receive_response("Check Configuring APN\n");
}

int configure_APN(int context_id){
	uint8_t command[256];
	snprintf((char *)command, sizeof(command), "AT+QICSGP=%d,%d,\"%s\",\"%s\",\"%s\",%d,0\r\n", context_id, 1, APN_NAME, APN_USERNAME, APN_PASSWD, APN_AUTHEN);
	send_AT_command((char*)command);
	osDelay(150);
	receive_response("CONFIGURE APN\n");
	char *first_pointer = NULL;
	char *second_pointer = NULL; 	
	while (first_pointer == NULL || second_pointer == NULL){
		check_configure_APN();
		osDelay(1300);
		first_pointer = strstr((char*)response, CHECK_RESPONSE);
		if(first_pointer != NULL){
			second_pointer = strstr(first_pointer+1, CHECK_RESPONSE);
		}
	}
	return 1;
}

void check_activate_context(){
	uint8_t command[128];
	snprintf((char *)command, sizeof(command), CHECK_ACTIVATE_CONTEXT);
	send_AT_command((char*)command);
	receive_response("CHECK Activate CONTEXT\n");
}

//void Delay_40s(void)
//{
//	for(int i = 0; i < 40; i++){
//    // Reset the timer counter to 0
//		__HAL_TIM_SET_COUNTER(&htim3, 0);
//
//		// Wait until the counter reaches 1000
//		while (__HAL_TIM_GET_COUNTER(&htim3) < 1000);
//	}
//	is_40s = 1;
//}
int activate_context(int context_id){
	uint8_t command[128];
	snprintf((char *)command, sizeof(command), "AT+QIACT=%d\r\n", context_id);
	send_AT_command((char*)command);
	osDelay(150);
	receive_response("Activate Context\r\n");
	char *first_pointer = NULL;
	char *second_pointer = NULL;
	int count_check = 0;
//	HAL_TIM_Base_Start(&htim3);
//	__HAL_TIM_SET_COUNTER(&htim3, 0);
	int count_error = 0;
	while(1){
//	while ((first_pointer == NULL || second_pointer == NULL)){
		check_activate_context();
		osDelay(300);
		if(count_check >= 50){
			count_check = 0;
//			return 0;
			break;
		}
		if (strstr((char*)response, "ERROR") != NULL){
			osDelay(500);
			count_error++;
			memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
			SIM_UART_ReInitializeRxDMA();
			send_AT_command((char *) command);
			osDelay(200);
		}

		if(count_error >= 3){
			memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
			SIM_UART_ReInitializeRxDMA();
			count_error = 0;
//			return 0;
			break;
		}

		receive_response("Check Activate Context\r\n");
		first_pointer = strstr((char*)response, CHECK_RESPONSE);
		if(first_pointer != NULL){
			second_pointer = strstr(first_pointer+1, CHECK_RESPONSE);
		}
		count_check++;
	}
//	HAL_TIM_Base_Start(&htim3);
	memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
	SIM_UART_ReInitializeRxDMA();
	return 1;
}

int deactivate_context(int context_id){
	uint8_t command[128];
	int count_check = 0;
	osDelay(100);
	snprintf((char *)command, sizeof(command), "AT+QIDEACT=%d\r\n", context_id);
	send_AT_command((char*)command);
	while(strstr((char *) response, CHECK_RESPONSE) == NULL){
		receive_response("DEACTIVATE CONTEXT\n");
		if (strstr((char *) response, "ERROR") != NULL){
			count_check = 0;
			memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
			SIM_UART_ReInitializeRxDMA();
			return 0;
		}
		if(count_check >= 20){
			count_check = 0;
			memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
			SIM_UART_ReInitializeRxDMA();
			return 0;
		}
		count_check++;
		osDelay(1200);
	}
	receive_response("DEACTIVATE CONTEXT\n");
	osDelay(100);
	memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
	SIM_UART_ReInitializeRxDMA();
	return 1;
}


int open_socket_service(int context_id, int connect_id, int local_port, int access_mode){
	const int timeout_seconds = 50; // Receive response each second
	//TODO: CHANGE timeout to 150 after testing
	int elapsed_time_ms = 0;
	uint8_t command[256];
	snprintf((char *)command, sizeof(command), "AT+QIOPEN=%d,%d,\"%s\",\"%s\",%d,%d,%d\r\n",context_id, connect_id, SERVICE_TYPE, IP_ADDRESS, REMOTE_PORT, local_port, access_mode);
	send_AT_command((char *) command);
	osDelay(100);
	char *first_pointer = NULL;
	//time_t start = time(NULL);
	int count_error = 0;
	uart_transmit_string(&huart1, (uint8_t *) "Init start TIME\n");
	while(elapsed_time_ms < timeout_seconds){
//	while(first_pointer == NULL && elapsed_time_ms < timeout_seconds){
		char output_elapsed[128];
		receive_response("Check OPEN socket service: \r\n");
		if (strstr((char *) response, "ERROR") != NULL){
			memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
			SIM_UART_ReInitializeRxDMA();
			count_error++;
			osDelay(500);
			send_AT_command((char *) command);
		}
		if(count_error >= 6){
			memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
			SIM_UART_ReInitializeRxDMA();
			count_error = 0;
			return 0;
		}
		first_pointer = strstr((char*)response, "+QIOPEN:");
		elapsed_time_ms++;
		snprintf(output_elapsed, 128, "Elapsed Time: %d\n", elapsed_time_ms);
		uart_transmit_string(&huart1, (uint8_t *)output_elapsed);
		osDelay(1000);
	}
	receive_response("Check OPEN socket service: \r\n");
	memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
	SIM_UART_ReInitializeRxDMA();
	
	if(first_pointer != NULL)
	{
		//AT+QISTATE=<query_type>,<connectID>
		snprintf((char *)command, sizeof(command), "AT+QISTATE=1,%d\r\n",connect_id);
		send_AT_command((char*) command);
		while(strstr((char *) response, CHECK_RESPONSE) == NULL){
			receive_response("Check SOCKET CONNECTION\n");
			if (strstr((char *) response, "ERROR") != NULL){
				memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
				SIM_UART_ReInitializeRxDMA();
				return 0;
			}
			osDelay(1000);
		}
		osDelay(100);
		memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
		SIM_UART_ReInitializeRxDMA();
		return 1;
	}
	else return 0;
}

//QPING command
int check_socket_connection(int context_ID){
	uint8_t command[256];
	int count_check =0;
	snprintf((char *)command, sizeof(command), "AT+QPING=%d,\"%s\"\r\n",context_ID, IP_ADDRESS);
	send_AT_command((char*)command);
	char *first_pointer = NULL;
	char *second_pointer = NULL;
	char *third_pointer = NULL;
	char *fourth_pointer = NULL;
	char *fifth_pointer = NULL;
	while(first_pointer == NULL || second_pointer == NULL || third_pointer == NULL || fourth_pointer == NULL || fifth_pointer == NULL){
		receive_response("Check SOCKET CONNECTION\n");

		if(count_check > 7){
			memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
			SIM_UART_ReInitializeRxDMA();
			count_check = 0;
			return 0;
		}
		first_pointer = strstr((char*)response, "+QPING:");
		if(first_pointer != NULL){
			 count_check = 0;
			 second_pointer = strstr(first_pointer+1, "+QPING:");
		}
		if(second_pointer != NULL){
			 count_check = 0;
			 third_pointer = strstr(second_pointer+1, "+QPING:");
		}
		if(third_pointer != NULL){
			 count_check = 0;
			 fourth_pointer = strstr(third_pointer+1, "+QPING:");
		}
		if(fourth_pointer != NULL){
			 count_check = 0;
			 fifth_pointer = strstr(fourth_pointer+1, "+QPING:");
		}
		count_check++;
		osDelay(1000);
	}
	receive_response("Check SOCKET CONNECTION\n");
	osDelay(100);
	memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
	SIM_UART_ReInitializeRxDMA();
	return 1;
}

// Function to format data into a hex string
int formatToHexString( const uint8_t* data, int length, char* output, int max_len, int writeIndex) {
    for (int i = 0; i < length; i++) {
        if (writeIndex + 2 >= max_len) {
            // Prevent buffer overflow
            return -1;
        }
        sprintf(output + writeIndex, "%02X", data[i]);
        writeIndex += 2;
    }
    return writeIndex;
}

int generateRegistrationMessage(const JT808_TerminalRegistration *data, char *hexString, int max_len) {
    int writeIndex = 0;
    writeIndex = formatToHexString(&(data->start_mask), sizeof(data->start_mask), hexString, max_len, writeIndex);
    writeIndex = formatToHexString(data->message_type, sizeof(data->message_type), hexString, max_len, writeIndex);
    writeIndex = formatToHexString(data->message_length, sizeof(data->message_length), hexString, max_len, writeIndex);
    writeIndex = formatToHexString(data->terminal_phone_number, sizeof(data->terminal_phone_number), hexString, max_len, writeIndex);
    writeIndex = formatToHexString(data->message_serial_number, sizeof(data->message_serial_number), hexString, max_len, writeIndex);
    writeIndex = formatToHexString(data->province_ID, sizeof(data->province_ID), hexString, max_len, writeIndex);
    writeIndex = formatToHexString(data->city_ID, sizeof(data->city_ID), hexString, max_len, writeIndex);
    writeIndex = formatToHexString(data->manufacturer_ID, sizeof(data->manufacturer_ID), hexString, max_len, writeIndex);
    writeIndex = formatToHexString(data->terminal_type, sizeof(data->terminal_type), hexString, max_len, writeIndex);
    writeIndex = formatToHexString(data->terminal_ID, sizeof(data->terminal_ID), hexString, max_len, writeIndex);
    writeIndex = formatToHexString(&(data->plate_color), sizeof(data->plate_color), hexString, max_len, writeIndex);
    writeIndex = formatToHexString(data->plate_no, sizeof(data->plate_no), hexString, max_len, writeIndex);
    writeIndex = formatToHexString(&(data->check_sum), sizeof(data->check_sum), hexString, max_len, writeIndex);
    writeIndex = formatToHexString(&(data->end_mask), sizeof(data->end_mask), hexString, max_len, writeIndex);

    if (writeIndex < 0) {
        // Handle error in formatting
        return -1;
    }
    return writeIndex;
}

int generateLocationInfoMessage(const JT808_LocationInfoReport* report, char* hexString, int max_len) {
   const uint8_t* fields[] = {
        &(report->start_mask), report->message_type, report->message_length,
        report->terminal_phone_number, report->terminal_serial_number, report->alarm,
        report->status, report->latitude, report->longitude, report->altitude,
        report->speed, report->direction, report->timestamp, report->mileage,
        report->oil, report->driving_record_speed, report->vehicle_id, report->signal,
        report->additional, &(report->end_mask)
    };
    int lengths[] = { 1, 2, 2, 6, 2, 4, 4, 4, 4, 2, 2, 2, 6, 6, 2, 2, 3, 1, 9, 1 };

    int writeIndex = 0;
    for (int i = 0; i < sizeof(fields) / sizeof(fields[0]); i++) {
        writeIndex = formatToHexString(fields[i], lengths[i], hexString, max_len, writeIndex);
        if (writeIndex < 0) return -1;
    }
    return writeIndex;
}



int login_to_server(int connect_id, const JT808_TerminalRegistration *reg_msg){
	uint8_t command[256];  // Increased buffer size
	char hexString[128] = {0};
	int count_check = 0;
	int result = generateRegistrationMessage(reg_msg, hexString, 128);
	if (result < 0) {
		uart_transmit_string(&huart1,(uint8_t*) "ERROR: FAILED to generate message string\n");
		return 1;
	}

	// Format the AT command with the hex message
	snprintf((char*)command, sizeof(command), "AT+QISENDEX=%d,\"%s\"\r\n", connect_id, hexString);
	//snprintf((char *)command, sizeof(command), "AT+QISENDEX=%d,\"%s\"\r\n", connect_id, message);
	send_AT_command((char*)command);

	while(1){
//	while(strstr((char *) response, "+QIURC") == NULL){
		char output_elapsed[128];
		//
		if(count_check >= 50){
			count_check = 0;
			memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
			SIM_UART_ReInitializeRxDMA();
			return 0;
		}
		if (strstr((char*) response, "ERROR") != NULL){
			memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
			SIM_UART_ReInitializeRxDMA();
			return 0;
		}
		count_check++;
		snprintf(output_elapsed, 128, "Elapsed Time: %d\n", count_check);
		uart_transmit_string(&huart1, (uint8_t *)output_elapsed);
		receive_response("Check sending to server\n");
		osDelay(100);
	}
	receive_response("Check sending to server\n");
	osDelay(100);
	memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
	SIM_UART_ReInitializeRxDMA();
	return 1;
}

int send_location_to_server(int connect_id, const JT808_LocationInfoReport *location_info){
	uint8_t command[256];  // Increased buffer size
	char hexString[131] = {0};
	int count_check = 0;
	int result = generateLocationInfoMessage(location_info, hexString, 131);
	if (result < 0) {
		uart_transmit_string(&huart1,(uint8_t*) "ERROR: FAILED to generate message string\n");
		return 1;
	}

	// Format the AT command with the hex message
	snprintf((char *) command, sizeof(command), "AT+QISENDEX=%d,\"%s\"\r\n", connect_id, hexString);
	//snprintf((char *)command, sizeof(command), "AT+QISENDEX=%d,\"%s\"\r\n", connect_id, message);
	send_AT_command((char*)command);
	while(1){
//	while(strstr((char *) response, "+QIURC") == NULL){
		char output_elapsed[128];

		if(count_check >= 50){
			count_check = 0;
			memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
			SIM_UART_ReInitializeRxDMA();
			return 0;
		}
		if (strstr((char*) response, "ERROR") != NULL){
			memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
			SIM_UART_ReInitializeRxDMA();
			return 0;
		}
		 if (strstr((char*)response, "closed") != NULL) {
			 memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
			 SIM_UART_ReInitializeRxDMA();
			 return 2;
		 }
		count_check++;
		snprintf(output_elapsed, 128, "Elapsed Time: %d\n", count_check);
		uart_transmit_string(&huart1, (uint8_t *)output_elapsed);
		receive_response("Check sending to server\n");
		osDelay(100);
	}
	receive_response("Check sending to server\n");
	osDelay(100);
	memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
	SIM_UART_ReInitializeRxDMA();
	return 1;
}


int check_data_sent_to_server(int connect_id){
	uint8_t command[256];
	int count_check = 0;
	uint8_t output[128];
	int count_timeout = 0;
	int count_error = 0;
	Debug_printf("\n\n---------------- IN QIRD: 0X1500h ------------------\n\n");
	snprintf((char *)command, sizeof(command), "AT+QIRD=%d,100\r\n", connect_id);
	send_AT_command((char*)command);
	while(1){
//	while(strstr((char *) response, "+QIRD") == NULL){
		char output_elapsed[128];
		if (strstr((char*)response, "ERROR") != NULL){
			Debug_printf("\n\n---------------- IN QIRD: 0X1500h: ERROR ------------------\n\n");
			memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
			SIM_UART_ReInitializeRxDMA();
			send_AT_command((char*)command);
			count_error++;
		}
		if(count_check >= 50){
			count_check = 0;
			memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
			SIM_UART_ReInitializeRxDMA();
			send_AT_command((char*)command);
			count_timeout++;

		}
		if(count_error == 3){
			Debug_printf("\n--------------------- SENDING CONNECTION ERROR ---------------------------\n");
			return 0;
		}
		if(count_timeout == 3){
			Debug_printf("\n--------------------- SENDING CONNECTION TIME OUT ---------------------------\n");
			return 0;
		}
		osDelay(100);
		snprintf(output_elapsed, 128, "Elapsed Time +QISEND: 0,0: %d\n", count_check);
		uart_transmit_string(&huart1, (uint8_t *)output_elapsed);
		count_check++;
		receive_response("Check received data from server\n");
	}
	receive_response("Check received data from server\n");
	char *token = strstr((char*)response, "+QIRD: ");
	int value = 0;

	if (token != NULL) {
		value = atoi(token + 7);  // Move past "+QIRD: " and convert to integer
	}
	snprintf((char*)output, 128, "\nNumber of character received: %d\n", value);
	uart_transmit_string(&huart1, output);
	if(value == 0) return 0;
	uart_transmit_string(&huart1, (uint8_t*) "OUT OF receive data from server\n");
	memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
	SIM_UART_ReInitializeRxDMA();
// --------------------------------------------------------------End of  QIRD ------------------------------------------------------------

	//Reset value
	count_check = 0;
	count_error = 0;
	count_timeout = 0;

	Debug_printf("\n\n---------------- IN QISEND: 0X0 ------------------\n\n");
	snprintf((char *)command, sizeof(command), "AT+QISEND=%d,0\r\n", connect_id);
	send_AT_command((char*)command);
	while(1){
//	while(strstr((char *) response, CHECK_RESPONSE) == NULL){
		char output_elapsed[128];
		if(count_check >= 50){
			count_check = 0;
			memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
			SIM_UART_ReInitializeRxDMA();
			send_AT_command((char*)command);
			count_timeout++;
		}
		if (strstr((char*) response, "ERROR") != NULL){
			Debug_printf("\n\n---------------- IN QISEND: 0X0: ERROR ------------------\n\n");
			memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
			SIM_UART_ReInitializeRxDMA();
			send_AT_command((char*)command);
			count_error++;
		}
		if(count_error == 3){
			Debug_printf("\n--------------------- SENDING CONNECTION ERROR ---------------------------\n");
			return 0;
		}
		if(count_timeout == 3){
			Debug_printf("\n--------------------- SENDING CONNECTION TIME OUT ---------------------------\n");
			return 0;
		}
		count_check++;
		osDelay(100);
		snprintf(output_elapsed, 128, "Elapsed Time +QISEND: 0,0: %d\n", count_check);
		uart_transmit_string(&huart1, (uint8_t *)output_elapsed);
		receive_response("Check sending to server\n");
	}

	receive_response("Check sending to server\n");
	int sentBytes, ackedBytes, unackedBytes;

	int result = sscanf((char*)response, "AT+QISEND=0,0 +QISEND: %d,%d,%d", &sentBytes, &ackedBytes, &unackedBytes);
	snprintf((char *)output, 128, "Lost Transmit BYTES: %d\n", unackedBytes);
	uart_transmit_string(&huart1, output);
	if (result == 3) {
		if (unackedBytes > 0) {
			return 0;
		}
	}
	memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
	SIM_UART_ReInitializeRxDMA();
	return 1;
}


int close_connection(int connect_id){
	uint8_t command[256];
	snprintf((char *)command, sizeof(command), "AT+QICLOSE=%d\r\n", connect_id);
	send_AT_command((char*)command);
	int count_check = 0;
	while(strstr((char *) response, CHECK_RESPONSE) == NULL){
		receive_response("Check CLOSING to server\n");
		if (strstr((char*)response, "ERROR") != NULL){
			memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
			SIM_UART_ReInitializeRxDMA();
			return 0;
		}

		if(count_check >= 5){
			count_check = 0;
			memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
			SIM_UART_ReInitializeRxDMA();
			return 0;
		}
		osDelay(1000);
		count_check++;
	}
	receive_response("Check CLOSING to server\n");
	osDelay(100);
	memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
	SIM_UART_ReInitializeRxDMA();
	return 1;
}


int extract_time_CCLK(uint8_t* message){
	int year, month, day, hour, minute, second, timezone;
	uint8_t output_buffer[128];

	sscanf((char*) message, "AT+CCLK?\r\n+CCLK: \"%2d/%2d/%2d,%2d:%2d:%2d%2d\"",
						&year, &month, &day, &hour, &minute, &second, &timezone);
	hour += 1;
	if (hour >= 24) {
		hour -= 24;
		day += 1;
		// Simplified example: Add code here to handle month/day overflow as needed
	}
	if(year < 24) return 0;
	rmc_jt.date.Yr = year;
	rmc_jt.date.Mon = month;
	rmc_jt.date.Day = day;
	rmc_jt.tim.hour = hour;
	rmc_jt.tim.min = minute;
	rmc_jt.tim.sec = second;
	set_time(hour, minute, second);
	set_date(year, month, day);
	snprintf((char*)output_buffer, 128, "Adjusted time to GMT+8: 20%02d/%02d/%02d, %02d:%02d:%02d\n", rmc_jt.date.Yr, rmc_jt.date.Mon, rmc_jt.date.Day, rmc_jt.tim.hour, rmc_jt.tim.min, rmc_jt.tim.sec);
	uart_transmit_string(&huart1, (uint8_t*) "RTC Time: ");
	uart_transmit_string(&huart1, (uint8_t*) "\n");
	uart_transmit_string(&huart1, output_buffer);
	return 1;
}


int getCurrentTime(){
	int count_check = 0;
	send_AT_command("AT+CCLK?\r\n");
	while(strstr((char *) response, CHECK_RESPONSE) == NULL){
		if(count_check >= 3 ){
			count_check = 0;
			memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
			SIM_UART_ReInitializeRxDMA();
			return 0;
		}
		receive_response("Get time\n");
		osDelay(100);
		count_check++;
	}
	receive_response("Get time\n");
	int result_extract = extract_time_CCLK(response);
	memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
	SIM_UART_ReInitializeRxDMA();
	if(result_extract)
		return 1;
	else return 0;
}


void receiveRMCDataWithAddrGSM(){
	uint8_t output_buffer[70];
	uart_transmit_string(&huart1, (uint8_t*)"\\Inside Receiving Data at GSM\n\n");
	osStatus_t status = osMessageQueueGet(RMC_MailQGSMId, &receivedDataGSM, NULL, 3000); // Wait for mail
	if(status == osOK){
		uart_transmit_string(&huart1, (uint8_t*)"\n\nReceived  ADDRESS Data at GSM: \n");
		uart_transmit_string(&huart1, (uint8_t*)"Address received from MAIL QUEUE: \n");
		current_addr_gsm = receivedDataGSM.address;
		if(checkAddrExistInQueue(current_addr_gsm, &result_addr_queue) == 0 || (current_addr_gsm >= end_addr_disconnect && current_addr_gsm <= (FLASH_END_ADDRESS - 0x100))){
//			current_addr_gsm = receivedDataGSM->address;
			Debug_printf("Saving data to variable to send to the server\n");
			Debug_printf("\n---------- Current data accepted at address: %08lx----------\n", current_addr_gsm);
			rmc_jt.lcation.latitude = receivedDataGSM.rmc.lcation.latitude;
			rmc_jt.lcation.longitude = receivedDataGSM.rmc.lcation.longitude;
			rmc_jt.speed = receivedDataGSM.rmc.speed;
			rmc_jt.course = receivedDataGSM.rmc.course;
			rmc_jt.lcation.NS = receivedDataGSM.rmc.lcation.NS;
			rmc_jt.lcation.EW = receivedDataGSM.rmc.lcation.EW;
			rmc_jt.isValid = receivedDataGSM.rmc.isValid;
			rmc_jt.date.Yr = receivedDataGSM.rmc.date.Yr;
			rmc_jt.date.Mon = receivedDataGSM.rmc.date.Mon;
			rmc_jt.date.Day = receivedDataGSM.rmc.date.Day;
			rmc_jt.tim.hour = receivedDataGSM.rmc.tim.hour;
			rmc_jt.tim.min = receivedDataGSM.rmc.tim.min;
			rmc_jt.tim.sec = receivedDataGSM.rmc.tim.sec;

			snprintf((char *)output_buffer, sizeof(output_buffer), "Time SENDING TO SERVER at GSM: %d:%d:%d\n", rmc_jt.tim.hour, rmc_jt.tim.min, rmc_jt.tim.sec);
			uart_transmit_string(&huart1, output_buffer);

			snprintf((char *)output_buffer, sizeof(output_buffer), "Date SENDING TO SERVER at GSM: %d/%d/%d\n", rmc_jt.date.Day, rmc_jt.date.Mon, rmc_jt.date.Yr);
			uart_transmit_string(&huart1, output_buffer);

			snprintf((char *)output_buffer, sizeof(output_buffer), "Location SENDING TO SERVER at GSM: %.6f %c, %.6f %c\n", rmc_jt.lcation.latitude, rmc_jt.lcation.NS, rmc_jt.lcation.longitude, receivedDataGSM.rmc.lcation.EW);
			uart_transmit_string(&huart1, output_buffer);

			snprintf((char *)output_buffer, sizeof(output_buffer),"Speed SENDING TO SERVER at GSM: %.2f, Course: %.2f, Valid: %d\n", rmc_jt.speed, rmc_jt.course, rmc_jt.isValid);
			uart_transmit_string(&huart1, output_buffer);

			received_RMC = 1;
		}
		else{
			Debug_printf("\n----------------Have sent data in this address successfully already: %08lx ----------------\n", receivedDataGSM.address);
		}
	}
	else{
		Debug_printf("There is no address mail left\n");
	}
}

int processUploadDataToServer(JT808_LocationInfoReport *location_info){
	int count_resend = 0;
	int count_check = 0;
	while(count_resend < 3){
		Debug_printf(" \n\n--------------------------- GOING TO SEND DATA TO SERVER: RESEND COUNT %d -----------------------\n\n", count_resend);
		send_location_to_server(0, location_info);

//		if(result_send_location){
		uart_transmit_string(&huart1, (uint8_t *)"Inside process: Check Sending Location Report\r\n");
		int result_check = check_data_sent_to_server(0);
		if(result_check){
			uart_transmit_string(&huart1, (uint8_t *)"Sending SUCCESS\n");
			receive_response("Check location report\n");
			memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
			SIM_UART_ReInitializeRxDMA();
			return 1;
		}
		else{
			uart_transmit_string(&huart1, (uint8_t *)"Sending ERROR (CHECKING SENDING RESULT ERROR)\n");
			memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
			SIM_UART_ReInitializeRxDMA();
			count_resend++;
		}
//		}
//		else if(result_send_location == 2){
//			Debug_printf("The connection to server is closed. \n");
//			count_resend++;
//		}
//		else{
//			uart_transmit_string(&huart1, (uint8_t *)"Sending ERROR (SENDING ERROR)\n");
//			count_resend++;
//		}
	}
	send_AT_command(FIRST_CHECK);
	while(strstr((char *) response, CHECK_RESPONSE) != NULL){
		receive_response("First check SIM MODULE\n");
		if(count_check > 10){
			Debug_printf("SIM MODULE BUG");
			memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
			return 2;
		}
		count_check++;
		osDelay(100);
	}
	receive_response("First check SIM MODULE\n");
	memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
	SIM_UART_ReInitializeRxDMA();
	return 0;
}

void StartGSM(void const * argument)
{
  /* USER CODE BEGIN StartGSM */
	Debug_printf("------------------------ Starting GSM: Pushing data to Server ------------------------");
	RingBufferDmaU8_initUSARTRx(&SIMRxDMARing, &huart3, response, SIM_RESPONSE_MAX_SIZE);
	
	JT808_TerminalRegistration reg_msg = create_terminal_registration();
	
	JT808_LocationInfoReport location_info = create_location_info_report();
	

	initQueue_GSM(&result_addr_queue);
//	initQueue_GSM(&mail_sent_queue);
//	size_t message_length;
//	uint8_t *message_array = {0};
//
//	size_t location_report_message_length;
//	uint8_t *location_report_message_array = {0};
	
	init_SIM_module();
	int isReady = 0;
	int process = 0;
	int countReconnect = 0;
	int is_set_uniqueID = 0;
//	uint8_t output_location[256];
//	char output_elapsed[128];
//	for(;;)
//	{
////		Debug_printf("\n\n\n--------------------- INSIDE GSM -----------------------\n\n\n");
//		osDelay(300);
//		switch(process){
//			//Wait for SIM module to start
//			case 0:
//				countReconnect = 0;
//				uart_transmit_string(&huart1, (uint8_t *)"First CHECK\r\n");
//				isReady = first_check_SIM();
//				if(isReady) process++;
//				else{
//					memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
//					SIM_UART_ReInitializeRxDMA();
//					uart_transmit_string(&huart1,(uint8_t*) "Rebooting SIM module\n");
//					reboot_SIM_module();
//				}
//				break;
//			case 1:
//				// Check status of SIM. Wait until SIM is ready
//				uart_transmit_string(&huart1, (uint8_t *)"Check EVERYTHING READY\r\n");
//				osDelay(100);
//				int check_SIM = check_SIM_ready();
//				if(is_set_uniqueID == 0){
//					memcpy(reg_msg.terminal_phone_number, terminal_phone_number, sizeof(terminal_phone_number));
//					memcpy(location_info.terminal_phone_number, terminal_phone_number, sizeof(terminal_phone_number));
//					is_set_uniqueID = 1;
//				}
//
//				osDelay(150);
//				if (check_SIM == 0){
//					memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
//					SIM_UART_ReInitializeRxDMA();
//					uart_transmit_string(&huart1,(uint8_t*) "Rebooting SIM module");
//					reboot_SIM_module();
//					process = 0;
//				}
//				else process++;
//				break;
//			case 2:
//				// Configure the PDP context
//				uart_transmit_string(&huart1, (uint8_t *)"Inside process: Configure PDP context\r\n");
//				memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
//				SIM_UART_ReInitializeRxDMA();
//				configure_APN(1);
//				process++;
//				memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
//				SIM_UART_ReInitializeRxDMA();
//				break;
//			case 3:
//				//Activate the PDP context.
//				uart_transmit_string(&huart1, (uint8_t *)"Inside process: Activate PDP context\r\n");
//				int receive_activate = activate_context(1);
//				if(receive_activate){
//					getCurrentTime();
//					uart_transmit_string(&huart1, (uint8_t*) "Activate PDP context successfully\n");
//					osDelay(200);
//					process++;
//					memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
//					SIM_UART_ReInitializeRxDMA();
//				}
//				else{
//					uart_transmit_string(&huart1, (uint8_t*) "Activate PDP Context Failed\n");
//					int receive_deactivate = deactivate_context(1);
//					memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
//					SIM_UART_ReInitializeRxDMA();
//					if (receive_deactivate) process = 1;
//					else {
//						memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
//						SIM_UART_ReInitializeRxDMA();
//						uart_transmit_string(&huart1,(uint8_t*) "Rebooting SIM module\n");
//						reboot_SIM_module();
//						process = 0;
//					}
//				}
//				break;
//			case 4:
//				//Open socket service
//				uart_transmit_string(&huart1, (uint8_t *)"Inside process: OPEN SOCKET SERVICE\r\n");
//				int received_res = open_socket_service(1, 0, 0, 0);
//				if(received_res){
////					uart_transmit_string(&huart1, (uint8_t*) "Connect to Server successfully\n");
////					//int result_check_connect = check_socket_connection(1);
////					if(result_check_connect == 1){
////						countReconnect++;
//						process++;
////					}
////					else{
////						memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
////						SIM_UART_ReInitializeRxDMA();
////						uart_transmit_string(&huart1,(uint8_t*) "Rebooting SIM module\n");
////						reboot_SIM_module();
////						process = 0;
////					}
//				}
//				else
//				{
//					uart_transmit_string(&huart1, (uint8_t*) "Connect to Server Failed\n");
//					int receive_deactivate = deactivate_context(1);
//					memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
//					SIM_UART_ReInitializeRxDMA();
//					if (receive_deactivate) process = 1;
//					else{
//						memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
//						SIM_UART_ReInitializeRxDMA();
//						uart_transmit_string(&huart1,(uint8_t*) "Rebooting SIM module\n");
//						reboot_SIM_module();
//						process = 0;
//					}
//				}
//				break;
//			case 5:
//				// REGISTER/LOGIN TO THE SERVER
//				uart_transmit_string(&huart1, (uint8_t *)"Inside process: Register/Login to the server.\r\n");
//				int result_send_login = login_to_server(0,&reg_msg);
//				if(result_send_login){
//					process++;
//				}
//				else process = 8;
//				break;
//			case 6:
//				//CHECK LOGIN TO SERVER
//				uart_transmit_string(&huart1, (uint8_t *)"Inside process: Check Register/Login\r\n");
//				int result_check_login = check_data_sent_to_server(0);
//				if(result_check_login){
//					receive_response("Check terminal register\n");
//					process++;
//				}
//				else process = 8;
//				break;
//			case 7:
//				is_in_sending = 1;
//				//Send Location
//				uart_transmit_string(&huart1, (uint8_t *)"Inside process: Send Location\r\n");
//				int result_get_current = getCurrentTime();
//				if(result_get_current == 0){
//					process++;
//					break;
//				}
//				while(1){
////					receiveRMCDataWithAddrGSM();
//					received_RMC = 1;
//					if(received_RMC == 1){
//						received_RMC = 0;
//						uart_transmit_string(&huart1, (uint8_t *)"RECEIVED RMC DATA AT GSM MODULE\n");
//						save_rmc_to_location_info(&location_info);
//						Debug_printf("Current stack address to be sent to the server: \n");
//						Debug_printf("Address going to send to server at GSM:(STACK FROM MAIL QUEUE)  %08lx\n", current_addr_gsm);
//
//
//
//						HAL_TIM_Base_Start(&htim3);
//						__HAL_TIM_SET_COUNTER(&htim3, 0);
////						if (osMutexAcquire(myMutex, osWaitForever) == osOK) {
//
//							uint32_t freeStack1 = osThreadGetStackSpace(GSMHandle);
//
//							Debug_printf("\n\n --------------Thread GSM %p is running low on stack: %04d bytes remaining----------\n\n",GSMHandle, freeStack1);
//							result_final = processUploadDataToServer(&location_info);
//							if(result_final == 1){
//								countReconnect = 0;
//								uart_transmit_string(&huart1, (uint8_t *)"Sending SUCCESS\n");
//								receive_response("Check location report\n");
//								memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
//								SIM_UART_ReInitializeRxDMA();
//
//								if(is_disconnect == 1 || is_using_flash == 1){
//									if(is_disconnect == 1){
//										end_addr_disconnect = current_addr_gsm;
//										in_getting_mail_stack = 1;
//										Debug_printf("End address of network outage. RECONNECTED SUCCESSFULLY: %08x\n", end_addr_disconnect);
//									}
//
//									// TODO: THe duration of checking data sent is 20s. WHAT IF IN THAT DURATION THE ACTUAL ADDRESS HAS BEEN SHIFT LEFT
//									Debug_printf("\n-----------ADDING current address to the result queue----------\n");
//									enqueue_GSM(&result_addr_queue, current_addr_gsm);
//									if(is_keep_up == 0) num_in_mail_sent++;
//
//									for (int i = 0; i < result_addr_queue.size-1; i++) {
//										int idx = (result_addr_queue.front + i) % MAX_SIZE;
//										//0x4F00 is the end threshold address.
//										if(result_addr_queue.data[idx] != (FLASH_END_ADDRESS - 0x100) && in_getting_mail_stack == 1){
//											is_keep_up = 1;
//										}
//									}
//									if(is_keep_up == 1 && in_getting_mail_stack == 1){
//										int count_stack = 0;
//										for (int i = 0; i < result_addr_queue.size-1; i++) {
//											int idx = (result_addr_queue.front + i) % MAX_SIZE;
//											//0x4F00 is the end threshold address.
//											if(result_addr_queue.data[idx] == (FLASH_END_ADDRESS - 0x100) && result_addr_queue.data[idx+1] == (FLASH_END_ADDRESS - 0x100)){
//												count_stack++;
//											}
//										}
//										for (int i = 0; i < result_addr_queue.size-1; i++) {
//											int idx = (result_addr_queue.front + i) % MAX_SIZE;
//											//0x4F00 is the end threshold address.
//											if(result_addr_queue.data[idx] == (FLASH_END_ADDRESS - 0x100) && result_addr_queue.data[idx+1] == (FLASH_END_ADDRESS - 0x100)){
//												result_addr_queue.data[idx] -= 128 * count_stack;
//												count_stack--;
//											}
//										}
//										in_getting_mail_stack = 0;
//										Debug_printf("\n\n-------------- HAVE SENT ALL THE STACKED DATA IN MAIL QUEUE ----------------\n\n");
//									}
//
//									Debug_printf("\n--------------RESULT ADDRESS QUEUE----------------\n");
//									printQueue_GSM(&result_addr_queue);
//
//									if(start_addr_disconnect >= end_addr_disconnect - 128 && checkAddrExistInQueue(end_addr_disconnect - 128, &result_addr_queue)){
//										Debug_printf("\n\n\n\n---------------END GETTING FROM FLASH-------------\n\n\n\n");
//										is_using_flash = 0;
//										clearQueue_GSM(&result_addr_queue);
//	//									clearQueue_GSM(&mail_sent_queue);
//										start_addr_disconnect = 0;
//										end_addr_disconnect = 0;
//										count_shiftleft = 0;
//										is_keep_up = 0;
//										Debug_printf("\n\n---------------- CLEAR THE MAIL QUEUE ---------------------\n\n");
//										while(1){
//											Debug_printf("Receiving MAIL\n");
//											osStatus_t status = osMessageQueueGet(RMC_MailQGSMId, &receivedDataGSM, NULL, 3000); // Wait for mail
//											if(status == osOK){
//												Debug_printf("Receiving MAIL For CLEARING: %08lx\n", receivedDataGSM.address);
//												// After processing, free the mail to clear it
//											}
//											else{
//												Debug_printf("Have cleared out all mail queue\n");
//												break;
//											}
//										}
//									}
//									else{
//										Debug_printf("\n\n------------------ USING FLASH TO PUSH TO SERVER -----------------\n\n");
//										is_using_flash = 1;
//									}
//									is_disconnect = 0;
//								}
//								is_pushing_data = 0;
//							}
//							else {
//								uart_transmit_string(&huart1, (uint8_t *)"Sending ERROR\n");
//								memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
//								SIM_UART_ReInitializeRxDMA();
//								if(is_disconnect == 0){
//									if(is_using_flash == 0){
//										start_addr_disconnect = current_addr_gsm;
//										Debug_printf("Saving start address of connection outage: %08x\n", start_addr_disconnect);
//									}
//									is_disconnect = 1;
//	//								is_using_flash = 0;
//								}
//
//								//Reconnect then disconnect case: "HAVE NOT COMPLETE Get and send all the data from the last disconnect phase"
//								if(is_using_flash == 1){
//									if(is_keep_up){
//										Debug_printf("\n-----------------BEFORE update the result address data: GSM --------------\n");
//										printQueue_GSM(&result_addr_queue);
//										Debug_printf("\n--------------- Update the result address data: GSM --------------\n");
//
//
//										//Delete all the address that has been getting from FLASH.
//										for (int i = 0; i < result_addr_queue.size; i++) {
//											int idx = (result_addr_queue.front + i) % MAX_SIZE;
//											if(result_addr_queue.data[idx] < start_addr_disconnect){
//												Debug_printf("CURRENT INDEX TO CHECK DELETING: %08lx", result_addr_queue.data[idx]);
//		//										result_addr_queue.data[idx] -= 128 * count_shiftleft;
//												deleteMiddle_GSM(&result_addr_queue, idx);
//												i--;
//											}
//										}
//										//This is the current address when fetching simultaneously with FLASH.
//										int count_shiftleft_dub = count_shiftleft;
//										for (int i = 0; i < result_addr_queue.size; i++) {
//											int idx = (result_addr_queue.front + i) % MAX_SIZE;
//											if(result_addr_queue.data[idx] == FLASH_END_ADDRESS-0x100){
//												result_addr_queue.data[idx] -= 128 * count_shiftleft_dub;
//												count_shiftleft_dub -= 1;
//											}
//											else{
//												result_addr_queue.data[idx] -= 128 * count_shiftleft;
//											}
//										}
//
//									}
//									else{
//										int count_stack = 0;
//										for (int i = 0; i < result_addr_queue.size-1; i++) {
//											int idx = (result_addr_queue.front + i) % MAX_SIZE;
//											//0x4F00 is the end threshold address.
//											if(result_addr_queue.data[idx] == (FLASH_END_ADDRESS - 0x100) && result_addr_queue.data[idx+1] == (FLASH_END_ADDRESS - 0x100)){
//												count_stack++;
//											}
//										}
//										for (int i = 0; i < result_addr_queue.size-1; i++) {
//											int idx = (result_addr_queue.front + i) % MAX_SIZE;
//											//0x4F00 is the end threshold address.
//											if(result_addr_queue.data[idx] == (FLASH_END_ADDRESS - 0x100) && result_addr_queue.data[idx+1] == (FLASH_END_ADDRESS - 0x100)){
//												result_addr_queue.data[idx] -= 128 * count_stack;
//												count_stack--;
//											}
//										}
//									}
//									printQueue_GSM(&result_addr_queue);
//									start_addr_disconnect -= 128 * count_shiftleft;
//									if(start_addr_disconnect < 0x3000) start_addr_disconnect = 0x3000;
//									end_addr_disconnect -= 128 *count_shiftleft;
//
//									count_shiftleft = 0;
//
//									//Clear osMail
//
//
//									Debug_printf("\n\n---------------- CLEAR THE MAIL QUEUE ---------------------\n\n");
//									while(1){
//										Debug_printf("Receiving MAIL\n");
//										osStatus_t status = osMessageQueueGet(RMC_MailQGSMId, &receivedDataGSM, NULL, 3000); // Wait for mail
//										if(status == osOK){
//											Debug_printf("Receiving MAIL: %08lx\n", receivedDataGSM.address);
//											if(is_keep_up == 0 && receivedDataGSM.address == 0x4F00){
//												for (int i = 0; i < num_in_mail_sent; i++) {
//													int idx = (result_addr_queue.rear - i + MAX_SIZE) % MAX_SIZE; // Calculate the index in reverse
//													result_addr_queue.data[idx] -= 128;
//												}
//											}
//											// After processing, free the mail to clear it
//										}
//										else{
//											Debug_printf("Have cleared out all mail queue\n");
//											break;
//										}
//									}
//									is_using_flash = 0;
//								}
//								is_pushing_data = 0;
//
//								if(result_final == 2){
//									Debug_printf("---------------------SIM ERROR ----------------------\n");
//									memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
//									SIM_UART_ReInitializeRxDMA();
//									uart_transmit_string(&huart1,(uint8_t*) "Rebooting SIM module\n");
//									reboot_SIM_module();
//									process = 0;
//									break;
//
//								} else{
//									Debug_printf("\n--------------------SENDING ERROR -----------------------\n");
//									process++;
//									break;
//								}
//							}
//							int period = __HAL_TIM_GET_COUNTER(&htim3)/1000;
//							Debug_printf("\n--------------------Sending to SERVER takes %d -----------------------\n\n",period);
//							Debug_printf("\n--------------------END OF SENDING SERVER --------------------------\n\n");
//							osDelay(200);
////							osMutexRelease(myMutex);
////						}
//					}
//				}
//				Debug_printf("\n--------------------END OF CASE 7 --------------------------\n\n");
//				break;
//
//			case 8:
//				//Close CONNECTION
//				uint32_t freeStack1 = osThreadGetStackSpace(GSMHandle);
//
//				Debug_printf("\n\n --------------Thread GSM %p is running low on stack: %04d bytes remaining----------\n\n",GSMHandle, freeStack1);
//				int result_close = close_connection(0);
//				if(result_close){
//					memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
//					SIM_UART_ReInitializeRxDMA();
//					if(countReconnect > 20){
//						uart_transmit_string(&huart1,(uint8_t*) "Rebooting SIM module\n");
//						reboot_SIM_module();
//						process = 0;
//					}
//					else{
//						uart_transmit_string(&huart1,(uint8_t*) "REOPEN CONNECTION TO SERVER\n");
//						process = 4;
//					}
//				}
//				else{
//					memset(response, 0x00, SIM_RESPONSE_MAX_SIZE);
//					SIM_UART_ReInitializeRxDMA();
//					uart_transmit_string(&huart1,(uint8_t*) "Rebooting SIM module\n");
//					reboot_SIM_module();
//					process = 0;
//				}
//				break;
//		}
//		if(is_in_sending == 0){
//			receiveRMCDataWithAddrGSM();
//		}
//		if(is_in_sending == 1){
//			is_in_sending = 0;
//		}
	while(1){
		Debug_printf("\nHello from GSM\n");
		uart_transmit_string(&huart1,(uint8_t*) "\n\n");
		receive_response("Check auto log\n");
		send_AT_command(FIRST_CHECK);
		while(strstr((char *) response, CHECK_RESPONSE) != NULL){
			receive_response("First check SIM MODULE\n");
		}
		send_AT_command(CONFIGURE_CS_SERVICE);

		osDelay(1000);
  }
  /* USER CODE END StartGSM */
}
