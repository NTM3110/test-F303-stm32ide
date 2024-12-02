#include "spi_flash.h"
#include <stdio.h>
#include <time.h>
#include "string.h"
#include "cmsis_os.h"
#include <inttypes.h>

#include "RS232-UART1.h"
#include "system_management.h"
#include "GPS.h"
#include "RTC.h"

#define FLASH_START_ADDRESS 0x3000
#define FLASH_END_ADDRESS   0x5000
#define SECTOR_SIZE         0x1000 // Assuming sector size is 4 KB
#define PAGE_SIZE           0x80

uint8_t sector_buffer[SECTOR_SIZE];
uint8_t next_page_buffer[PAGE_SIZE];

extern UART_HandleTypeDef huart1;
uint32_t address_tax = 0x1000;
uint32_t address_rmc = 0x3000;


uint32_t current_addr = 0x3000;
int is_erased_tax = 0;
int is_erased_rmc = 0;

uint8_t flashBufferTaxReceived[128];
uint8_t flashBufferRMCReceived[128];
uint8_t taxBufferDemo[128];
uint8_t rmcBufferDemo[128];

RMCSTRUCT rmc_flash;
GSM_MAIL_STRUCT mail_gsm;

int W25_ChipErase(void)
{
	int retval;
	uint8_t cmd = {W25_CMD_CHIP_ERASE};
	W25_WriteEnable(); // Make sure we can write...
	W25_CS_ENABLE(); // Drive Winbond chip select, /CS low
	retval = HAL_SPI_Transmit(&hspi1, &cmd , sizeof(cmd ), TIMEOUT); // Send Chip Erase command
	W25_CS_DISABLE();
	W25_DelayWhileBusy(CHIP_ERASE_TIMEOUT);
	return retval;
} // W25_ChipErase()

uint8_t W25_ReadStatusReg1(void) {
	uint8_t cmd = W25_CMD_READ_STATUS_REG_1;
	uint8_t status_reg1;
	int retval;
	W25_CS_ENABLE(); // Drive Winbond chip select, /CS low
	retval = HAL_SPI_Transmit(&hspi1, &cmd , sizeof(cmd), TIMEOUT); // Send Read Status Reg 1 command
	if(retval == HAL_OK)
	retval = HAL_SPI_Receive(&hspi1, &status_reg1, sizeof(status_reg1), TIMEOUT);
	W25_CS_DISABLE();
	return retval == HAL_OK ? status_reg1:0xFF; // return 0xFF if error
} // W25_ReadStatusReg1()

int W25_Busy(void)
{
	return (W25_ReadStatusReg1() & W25_STATUS1_BUSY);
}

int W25_DelayWhileBusy(uint32_t msTimeout)
{
	uint32_t initial_count = HAL_GetTick();
	int busy;
	uint32_t deltaticks;
	uint32_t count = 0;
	do {
	busy = W25_Busy();
	deltaticks = HAL_GetTick() - initial_count;
	count++;
	} while(busy && deltaticks < msTimeout);
	int retval = busy ? HAL_TIMEOUT:HAL_OK;
	return retval;
}
void W25_Reset(){
	W25_CS_ENABLE();
	W25_CS_DISABLE();
	W25_CS_ENABLE();
	W25_CS_DISABLE();
	W25_CS_ENABLE();
	W25_CS_DISABLE();
}

int W25_ReadJedecID() {
	int retval;
	uint8_t idcmd = W25_CMD_READ_JEDEC_ID;
	uint8_t jdec_id[4];
	char result[11];
	W25_CS_ENABLE(); // Drive Winbond chip select, /CS low
	retval = HAL_SPI_TransmitReceive(&hspi1, &idcmd, jdec_id, 4, 4000);
	W25_CS_DISABLE();
	char spi_flash_intro[] = "Flash ID received: ";
	HAL_UART_Transmit(&huart1, (uint8_t*) spi_flash_intro, strlen(spi_flash_intro), 1000);

	sprintf(result, "%02X, %02X, %02X", jdec_id[1], jdec_id[2], jdec_id[3]);
	HAL_UART_Transmit(&huart1, (uint8_t*) result, 11, 1000);
	HAL_UART_Transmit(&huart1, (uint8_t*)"\r", 1, 1000);
	return retval;
} // W25_ReadJEDECID()

int W25_ReadMD(uint8_t *buf, int bufSize) {
	int retval;
	uint8_t cmddata[4] = {0x90,0,0,0};
	if(bufSize < W25_UNIQUE_ID_BUF_SIZE)
	return HAL_ERROR; // buffer too small
	W25_CS_ENABLE(); // Drive Winbond chip select, /CS low
	retval = HAL_SPI_Transmit(&hspi1, cmddata , sizeof(cmddata ), TIMEOUT); // Send the ID command
	if(retval == HAL_OK)
	retval = HAL_SPI_Receive(&hspi1, buf, W25_UNIQUE_ID_BUF_SIZE, TIMEOUT);
	W25_CS_DISABLE();
	return retval;
} // W25_ReadUniqueID()

int W25_ReadUniqueID(uint8_t *buf, int bufSize) {
	int retval;
	uint8_t cmddata[5] = {W25_CMD_READ_UNIQUE_ID,0,0,0,0};
	if(bufSize < W25_UNIQUE_ID_BUF_SIZE)
	return HAL_ERROR; // buffer too small
	W25_CS_ENABLE(); // Drive Winbond chip select, /CS low
	retval = HAL_SPI_Transmit(&hspi1, cmddata , sizeof(cmddata ), TIMEOUT); // Send the ID command
	if(retval == HAL_OK)
	retval = HAL_SPI_Receive(&hspi1, buf, W25_UNIQUE_ID_BUF_SIZE, TIMEOUT);
	W25_CS_DISABLE();

	return retval;
} // W25_ReadUniqueID()


int W25_WriteEnable(void) {
	uint8_t cmd = W25_CMD_WRITE_ENABLE;
	W25_CS_ENABLE(); // Drive Winbond chip select, /CS low
	int retval = HAL_SPI_Transmit(&hspi1, &cmd , sizeof(cmd), TIMEOUT); // Send Write Enable command
	W25_CS_DISABLE();
	return retval;
} // W25_WriteEnable()

int W25_SectorErase(uint32_t address)
{
	int retval;
	uint8_t cmdaddr[4] = {W25_CMD_SECTOR_ERASE,address>>16,address>>8,address};
	W25_WriteEnable(); // Make sure we can write...
	W25_CS_ENABLE(); // Drive Winbond chip select, /CS low
	retval = HAL_SPI_Transmit(&hspi1, cmdaddr , sizeof(cmdaddr ), TIMEOUT); // Send Sector Erase command with address
	W25_CS_DISABLE();
	W25_DelayWhileBusy(SECTOR_ERASE_TIMEOUT);
	return retval;
} // W25_SectorErase()

int W25_PageProgram(uint32_t address, uint8_t *buf, uint32_t count)
{
	int retval = HAL_OK;
	W25_WriteEnable(); // Make sure we can write...
	while(count) {
		uint8_t cmdaddr[4] = {W25_CMD_PAGE_PROGRAM,address>>16,address>>8,address};
		uint32_t space_left_in_page = 0x100 - (address & 0xFF);
		uint32_t count_this_pass = count <= space_left_in_page? count:space_left_in_page;
		W25_CS_ENABLE(); // Drive Winbond chip select, /CS low
		retval = HAL_SPI_Transmit(&hspi1, cmdaddr , sizeof(cmdaddr ), TIMEOUT); // Send Page Program command with address
		if(retval == HAL_OK)
		  retval = HAL_SPI_Transmit(&hspi1, buf, count_this_pass, TIMEOUT); // Write page data
		W25_CS_DISABLE();
		count -= count_this_pass;
		address += count_this_pass;
		buf += count_this_pass;
		W25_DelayWhileBusy(PAGE_PROGRAM_TIMEOUT);
	}
	return retval;
} 

// Winbond 8.2.6 Read Data (03h)
// The only limit for quantity of data is memory / device size
int W25_ReadData(uint32_t address, uint8_t *buf, int bufSize)
{
	int retval;
	uint8_t cmdaddr[4] = {W25_CMD_READ_DATA,address>>16,address>>8,address};
	//printf("+%s(Addr 0x%06X, buf 0x%08X, Len 0x%04X)\r\n",__func__,address,buf,bufSize);
	W25_CS_ENABLE(); // Drive Winbond chip select, /CS low
	retval = HAL_SPI_Transmit(&hspi1, cmdaddr , sizeof(cmdaddr), 500); // Send Read Data command with address
	if(retval != HAL_OK) {
		return retval;
	}
	//memset(buf,0,bufSize); // Buffer is transmitted during receive
	retval = HAL_SPI_Receive(&hspi1, buf, bufSize, 2000); // need longer time-outs when using slow SPI clock
	if(retval != HAL_OK)

	W25_CS_DISABLE();

	return retval;
} // W25_ReadData()


// Function to check if a page contains valid data
int IsPageValid(uint8_t *page) {
    char *last_semicolon = strrchr((char *)page, ';'); // Find the last semicolon
    if (!last_semicolon) {
        return 0; // No semicolon found, invalid page
    }

    char *last_param = last_semicolon + 1; // Last parameter starts after the last semicolon
    size_t len = strlen(last_param);

    // Ensure the last parameter is exactly 6 characters and numeric
    if (len < 6) {
        return 0; // Too short to be valid
    }

    for (int i = 0; i < 6; ++i) {
        if ((last_param[i] < '0' || last_param[i] > '9' ) &&
                (last_param[i] < 'a' || last_param[i] > 'f')) {
            return 0; // Not numeric
        }
    }

    return 1; // Page contains valid data
}


// Function to update the last parameter of the page (address)
void UpdatePageAddress(uint8_t *page, uint32_t new_address) {
    char new_address_str[7];
    snprintf(new_address_str, sizeof(new_address_str), "%06lx", new_address);
    memcpy(page + strlen((char *)page) - 6, new_address_str, 6); // Overwrite last 6 characters
}

int W25_ShiftLeftFlashDataByPage(uint32_t src_addr, uint32_t dest_addr) {
	int loop_time = 0;
	if(src_addr == 0x3000 && dest_addr == 0x3000){
		loop_time = 1;
	}
	loop_time = (src_addr - dest_addr) / 128;
    uint32_t current_sector_start = FLASH_START_ADDRESS;
    for(size_t i = 0; i < loop_time; i++){
		while (current_sector_start < FLASH_END_ADDRESS) {
			uint32_t next_sector_start = current_sector_start + SECTOR_SIZE;

			W25_Reset();
			// Step 1: Read the entire current sector into the buffer
			if (W25_ReadData(current_sector_start, sector_buffer, SECTOR_SIZE) != HAL_OK) {
				Debug_printf("READING ALL THE SECTOR: ERROR\n");
				return HAL_ERROR; // Exit if read fails
			}
			else{
				char spi_flash_data_intro[] = "First page of Flash DATA at Reading All Sector: \n";
				HAL_UART_Transmit(&huart1, (uint8_t*) spi_flash_data_intro, strlen(spi_flash_data_intro), 1000);
				HAL_UART_Transmit(&huart1, sector_buffer, 128, 1000);
				Debug_printf("\n\n");
			}

			// Step 2: Read the first page of the next sector (if exists)
			if (next_sector_start < FLASH_END_ADDRESS) {
				W25_Reset();
				if (W25_ReadData(next_sector_start, next_page_buffer, PAGE_SIZE) != HAL_OK) {
					Debug_printf("READING FIRST PAGE OF THE NEXT SECTOR: ERROR\n");
					return HAL_ERROR; // Exit if read fails
				}
				else{
					char spi_flash_data_intro[] = "First page of Flash DATA at reading NEXT SECTOR: \n";
					HAL_UART_Transmit(&huart1, (uint8_t*) spi_flash_data_intro, strlen(spi_flash_data_intro), 1000);
					HAL_UART_Transmit(&huart1, next_page_buffer, 128, 1000);
					Debug_printf("\n\n");
				}
			}

			// Step 3: Shift the current sector's data left within the buffer
			for (uint32_t offset = 0; offset < SECTOR_SIZE - PAGE_SIZE; offset += PAGE_SIZE) {
				if (IsPageValid(sector_buffer + offset + PAGE_SIZE)) {
					Debug_printf("------VALID PAGE at %08lx-------\n", offset+PAGE_SIZE);
					char spi_flash_data_intro[] = "Valid Page Data: \n";
					HAL_UART_Transmit(&huart1, (uint8_t*) spi_flash_data_intro, strlen(spi_flash_data_intro), 1000);
					HAL_UART_Transmit(&huart1, sector_buffer + offset+ PAGE_SIZE, 128, 1000);
					Debug_printf("\n");
					memcpy(sector_buffer + offset, sector_buffer + offset + PAGE_SIZE, PAGE_SIZE);
					UpdatePageAddress(sector_buffer + offset, current_sector_start + offset);
				} else {
					Debug_printf("-------INVALID PAGE at %08lx-------\n", offset+PAGE_SIZE);
					char spi_flash_data_intro[] = "Invalid Page Data: \n";
					HAL_UART_Transmit(&huart1, (uint8_t*) spi_flash_data_intro, strlen(spi_flash_data_intro), 1000);
					HAL_UART_Transmit(&huart1, sector_buffer + offset +PAGE_SIZE, 128, 1000);
					Debug_printf("\n");
					memset(sector_buffer + offset, 0xFF, PAGE_SIZE); // Invalidate the page
				}
			}

			// Step 4: Move the next sector's first page into the last page of the current sector
			if (next_sector_start < FLASH_END_ADDRESS && IsPageValid(next_page_buffer)) {
				memcpy(sector_buffer + (SECTOR_SIZE - PAGE_SIZE), next_page_buffer, PAGE_SIZE);
				UpdatePageAddress(sector_buffer + (SECTOR_SIZE - PAGE_SIZE), current_sector_start + (SECTOR_SIZE - PAGE_SIZE));
			} else {
			   // memset(sector_buffer + (SECTOR_SIZE - PAGE_SIZE), 0xFF, PAGE_SIZE); // Invalidate the last page
			}

			// Step 5: Erase the current sector
			W25_Reset();
			if (W25_SectorErase(current_sector_start) != HAL_OK) {
				Debug_printf("Erase ALL CURRENT SECTOR: ERROR\n");
				return HAL_ERROR; // Exit if erase fails
			}

			// Step 6: Write the updated buffer back to the current sector
			for (uint32_t offset = 0; offset < SECTOR_SIZE; offset += PAGE_SIZE) {
				W25_Reset();
				if (W25_PageProgram(current_sector_start + offset, sector_buffer + offset, PAGE_SIZE) != HAL_OK) {
					Debug_printf("PAGE PROGRAM: ERROR at page offset %08lx\n", offset);
					return HAL_ERROR; // Exit if write fails
				}
			}

			// Move to the next sector
			current_sector_start = next_sector_start;
		}
    }

    return HAL_OK;
}

void receiveTaxData(void) {
//	uint8_t output_buffer[200];
	int k = 0;
	int j;
    osEvent evt = osMailGet(tax_MailQId, 2000); // Wait for mail
    if (evt.status == osEventMail) {
		TAX_MAIL_STRUCT *receivedData = (TAX_MAIL_STRUCT *)evt.value.p;
		uart_transmit_string(&huart1, (uint8_t*)"Received  TAX Data: \n");
		// Process received data (e.g., display, log, or store data)
		uart_transmit_string(&huart1, receivedData->data);
		for(size_t i = 0; i < 128; i++){
			taxBufferDemo[i] = receivedData->data[i];
			if(receivedData->data[i] != 0x00 && receivedData->data[i+1] == 0x00) k = i;
		}
		osMailFree(tax_MailQId, receivedData); // Free memory after use
		char addr_out[10];
		sprintf(addr_out, "%08lx", address_tax);
		HAL_UART_Transmit(&huart1, (uint8_t*) addr_out, 8, 1000);
		HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 1, 1000);
		k++;
		taxBufferDemo[k] = ';';
		for(size_t idx = 6; idx > 0 ; idx--){
			k++;
			taxBufferDemo[k] = addr_out[8 - idx];
		}

		for (j=0;j<110-k-1;j++)
		{
			taxBufferDemo[j+k+1]=0x00;
		}
		char tax_buffer_intro[] = "Tax Buffer SAVED SPI FLASH: ";
		HAL_UART_Transmit(&huart1, (uint8_t*) tax_buffer_intro, strlen(tax_buffer_intro), 1000);
		HAL_UART_Transmit(&huart1, taxBufferDemo, sizeof(taxBufferDemo), 100);
		HAL_UART_Transmit(&huart1, (uint8_t*)"\n", 1, 100);

		W25_Reset();
		if (is_erased_tax == 0){
			W25_SectorErase(address_tax);
			is_erased_tax = 1;
		}
		W25_Reset();
		W25_PageProgram(address_tax, taxBufferDemo, 128);
		current_addr = address_tax;
		address_tax += 128;
		HAL_Delay(1000);
		memset(flashBufferTaxReceived, 0x00,128);
	}
}

void receiveResultAddress(){
	osEvent evt = osMailGet(result_MailQGSMId, 2000); // Wait for mail
	if (evt.status == osEventMail) {
		uint32_t *receivedData = (uint32_t *) evt.value.p;
		result_address = *receivedData;
		osMailFree(result_MailQGSMId, receivedData);
		Debug_printf("\n\nThe result address received at SPI FLASH from GSM that the data in this ADDRESS has been sent successfully: %08lx\n\n", result_address);
	}
}

uint32_t calculate_epoch_time_utc(DATE *date, TIME *time) {
    struct tm timeinfo;
    // Set up time structure
    timeinfo.tm_year = date->Yr - 1900; // - 1900 + 2000
    timeinfo.tm_mon = date->Mon - 1;
    timeinfo.tm_mday = date->Day;
    timeinfo.tm_hour = time->hour;
    timeinfo.tm_min = time->min;
    timeinfo.tm_sec = time->sec;
    timeinfo.tm_isdst = -1; // Let mktime determine DST if necessary

    // Get the local epoch time and then adjust for timezone offset
    time_t local_epoch = mktime(&timeinfo);
    return (uint32_t)(local_epoch + 25200); // Subtract timezone offset
}



void format_rmc_data(RMCSTRUCT *rmc_data, char *output_buffer, size_t buffer_size) {
	//uart_transmit_string(&huart1, (uint8_t*) "Format RMC data");
    uint32_t epoch_time = calculate_epoch_time_utc(&rmc_data->date, &rmc_data->tim);

    // Format all fields in a single line with semicolon separation, including date
    snprintf(output_buffer, buffer_size, "%d;%d;%d;%d;%d;%d;%.6f;%c;%.6f;%c;%.1f;%.1f;%s;%lu", rmc_data->date.Yr, rmc_data->date.Mon, rmc_data->date.Day, rmc_data->tim.hour, rmc_data->tim.min, rmc_data->tim.sec, rmc_data->lcation.latitude, rmc_data->lcation.NS, rmc_data->lcation.longitude, rmc_data->lcation.EW, rmc_data->speed, rmc_data->course, rmc_data->isValid ? "Valid" : "Invalid", epoch_time);
}


void saveRMC(){
	Debug_printf("\n\n Inside SAVING RMC TO FLASH \n\n");
	int k = 0;
	int j = 0;
	W25_Reset();
	if (is_erased_rmc == 0){
		W25_SectorErase(address_rmc);
		is_erased_rmc = 1;
	}
	
	for(size_t i = 0; i < 128; i++){
		if(rmcBufferDemo[i] != 0x00 && rmcBufferDemo[i+1] == 0x00){
			k = i;
			break;
		}
	}
	char addr_out[10];
	sprintf(addr_out, "%08lx", address_rmc);
	HAL_UART_Transmit(&huart1, (uint8_t*) addr_out, 8, 1000);
	HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 1, 1000);
	
	k++;
	rmcBufferDemo[k] = ';';
	for(size_t idx = 6; idx > 0 ; idx--){
		k++;
		rmcBufferDemo[k] = addr_out[8 - idx];
	}
	
	for (j=0;j<110-k-1;j++)
	{
		rmcBufferDemo[j+k+1]=0x00;
	}


	W25_Reset();
	W25_PageProgram(address_rmc, rmcBufferDemo, 128);
	uart_transmit_string(&huart1, (uint8_t*) "Buffer before saving to FLASH: ");
	uart_transmit_string(&huart1, rmcBufferDemo);
	current_addr = address_rmc;
	address_rmc+=128;
	if(result_address % 0x1000 && result_address > 0){
		Debug_printf("\n\nErasing SECTOR IN ADVANCE\n");
		address_rmc -=  0x1000;
		W25_ShiftLeftFlashDataByPage(result_address, result_address - 0x1000);
		W25_SectorErase(result_address-0x1000);
	}
	if(current_addr >= 0x8F80){
		//TODO: Reach end address so shift left all the address before result_address then if reach end address again. Shift left everytime get rmc again.

		address_rmc = 0x3000;
		W25_SectorErase(address_rmc);
	}
	HAL_Delay(1000);
	Debug_printf("\n");
	memset(flashBufferRMCReceived, 0x00,128);
}

void sendRMCDataToGSM(RMCSTRUCT *rmcData){
	if(rmcData->date.Yr >= 24){
		HAL_UART_Transmit(&huart1, (uint8_t*) "\n\n\nSENDING RMC TO GSM\n\n",  strlen("\n\n\nSENDING RMC TO GSM\n\n") , HAL_MAX_DELAY);
		RMCSTRUCT *mail = (RMCSTRUCT *)osMailAlloc(RMC_MailQLEDId, osWaitForever); // Allocate memory for mail
		if (mail != NULL) {
			*mail = *rmcData; // Copy data into allocated memory
			osMailPut(RMC_MailQLEDId, mail); // Put message in queue
		}
	}
}

void sendRMCDataWithAddrToGSM(GSM_MAIL_STRUCT *mail_data){
	if(mail_data->rmc.date.Yr >= 24){
		HAL_UART_Transmit(&huart1, (uint8_t*) "\n\n\nSENDING RMC with Addr TO GSM\n\n",  strlen("\n\n\nSENDING RMC with Addr TO GSM\n\n") , HAL_MAX_DELAY);
		GSM_MAIL_STRUCT *mail = (GSM_MAIL_STRUCT *)osMailAlloc(RMC_MailQGSMId, osWaitForever); // Allocate memory for mail
		if (mail != NULL) {
			*mail = *mail_data; // Copy data into allocated memory
			osMailPut(RMC_MailQGSMId, mail); // Put message in queue
		}
	}
}


void sendAddresstoGSM(){
	if(rmc_flash.date.Yr >= 24){
		uint32_t *mail = (uint32_t *)osMailAlloc(addr_MailQGSMId, osWaitForever);
		if (mail != NULL) {
			*mail = current_addr; // Copy the data
			osMailPut(addr_MailQGSMId, mail); // Send the mail
		}
	}
}

void receiveRMCDataFromGPS(void) {
	static int countRMCReceived = 0;
	uint8_t output_buffer[70];

	// Wait until there are at least 10 messages in the queue
	uart_transmit_string(&huart1, (uint8_t*)"Inside Receiving RMC Data SPI FLASH\n");
	osEvent evt = osMailGet(RMC_MailQFLASHId, osWaitForever); // Wait for mail
	if(evt.status == osEventMail){
		uart_transmit_string(&huart1, (uint8_t*)"\nReceived  RMC Data SPI FLASH: \n");
		RMCSTRUCT *receivedData = (RMCSTRUCT *)evt.value.p;
//		 Process received data (e.g., display, log, or store data)
//		snprintf((char *)output_buffer, sizeof(output_buffer), "Time Received FLASH: %d:%d:%d\n", receivedData->tim.hour, receivedData->tim.min, receivedData->tim.sec);
//		uart_transmit_string(&huart1, output_buffer);
//
//		snprintf((char *)output_buffer, sizeof(output_buffer), "Date Received FLASH : %d/%d/%d\n", receivedData->date.Day, receivedData->date.Mon, receivedData->date.Yr);
//		uart_transmit_string(&huart1, output_buffer);
//
//		snprintf((char *)output_buffer, sizeof(output_buffer), "Location Received FLASH: %.6f %c, %.6f %c\n", receivedData->lcation.latitude, receivedData->lcation.NS, receivedData->lcation.longitude, receivedData->lcation.EW);
//		uart_transmit_string(&huart1, output_buffer);
//
//		snprintf((char *)output_buffer, sizeof(output_buffer),"Speed FLASH: %.2f, Course: %.2f, Valid: %d\n", receivedData->speed, receivedData->course, receivedData->isValid);
//		uart_transmit_string(&huart1, output_buffer);
//		uart_transmit_string(&huart1, (uint8_t*)"\n\n");

		//Sending DATA to GSM
		rmc_flash.lcation.latitude = receivedData->lcation.latitude;
		rmc_flash.lcation.longitude = receivedData->lcation.longitude;
		rmc_flash.speed = receivedData->speed;
		rmc_flash.course = receivedData->course;
		rmc_flash.lcation.NS = receivedData->lcation.NS;
		rmc_flash.lcation.EW = receivedData->lcation.EW;
		rmc_flash.isValid = receivedData->isValid;
		rmc_flash.tim.hour = receivedData->tim.hour;
		rmc_flash.tim.min = receivedData->tim.min;
		rmc_flash.tim.sec = receivedData->tim.sec;
		rmc_flash.date.Yr = receivedData->date.Yr;
		rmc_flash.date.Mon = receivedData->date.Mon;
		rmc_flash.date.Day = receivedData->date.Day;


		get_RTC_time_date(&rmc_flash);
//		uart_transmit_string(&huart1, (uint8_t*)"RMC Data  Saved GSM\n");
//		// Process received data (e.g., display, log, or store data)
		snprintf((char *)output_buffer, sizeof(output_buffer), "\n\nTime Received from GPS AT SPI FLASH: %d:%d:%d\n", rmc_flash.tim.hour, rmc_flash.tim.min, rmc_flash.tim.sec);
		uart_transmit_string(&huart1, output_buffer);

		snprintf((char *)output_buffer, sizeof(output_buffer), "Date Received FROM GPS AT SPI FLASH : %d/%d/%d\n", rmc_flash.date.Day, rmc_flash.date.Mon, rmc_flash.date.Yr);
		uart_transmit_string(&huart1, output_buffer);

		snprintf((char *)output_buffer, sizeof(output_buffer), "Location Received FROM GPS AT SPI FLASH: %.6f %c, %.6f %c\n", rmc_flash.lcation.latitude, rmc_flash.lcation.NS, rmc_flash.lcation.longitude, rmc_flash.lcation.EW);
		uart_transmit_string(&huart1, output_buffer);

		snprintf((char *)output_buffer, sizeof(output_buffer),"Speed FROM GPS AT SPI FLASH: %.2f, Course: %.2f, Valid: %d\n\n\n", rmc_flash.speed, rmc_flash.course, rmc_flash.isValid);
		uart_transmit_string(&huart1, output_buffer);

		format_rmc_data(&rmc_flash,(char*) rmcBufferDemo, 128);

		if(rmc_flash.date.Yr >= 24 && countRMCReceived == 9){
			saveRMC();
			//sendRMCDataToGSM(&rmc_flash);
			mail_gsm.rmc.lcation.latitude = rmc_flash.lcation.latitude;
			mail_gsm.rmc.lcation.longitude = rmc_flash.lcation.longitude;
			mail_gsm.rmc.speed = rmc_flash.speed;
			mail_gsm.rmc.course = rmc_flash.course;
			mail_gsm.rmc.lcation.NS = rmc_flash.lcation.NS;
			mail_gsm.rmc.lcation.EW = rmc_flash.lcation.EW;
			mail_gsm.rmc.isValid = rmc_flash.isValid;
			mail_gsm.rmc.tim.hour = rmc_flash.tim.hour;
			mail_gsm.rmc.tim.min = rmc_flash.tim.min;
			mail_gsm.rmc.tim.sec = rmc_flash.tim.sec;
			mail_gsm.rmc.date.Yr = rmc_flash.date.Yr;
			mail_gsm.rmc.date.Mon = rmc_flash.date.Mon;
			mail_gsm.rmc.date.Day = rmc_flash.date.Day;
			mail_gsm.address = current_addr;
//			sendAddresstoGSM();
			sendRMCDataWithAddrToGSM(&mail_gsm);
			countRMCReceived = 0;
		}
		osMailFree(RMC_MailQFLASHId, receivedData);
		// Free memory after use
		if(rmc_flash.date.Yr >= 24)
			countRMCReceived++;
	}
}


void StartSpiFlash(void const * argument)
{
  /* USER CODE BEGIN StartSpiFlash */
  /* Infinite loop */
	current_addr = address_rmc;

	osMailQDef(GSM_MailQ, 128, GSM_MAIL_STRUCT);
	RMC_MailQGSMId = osMailCreate(osMailQ(GSM_MailQ), NULL);


	for(;;){
		osDelay(1500);
		//uart_transmit_string(&huart1, (uint8_t*) "INSIDE SPI FLASH\n");
		W25_Reset();
		W25_ReadJedecID();
		W25_Reset();
		receiveResultAddress();
		W25_ReadData(current_addr, flashBufferRMCReceived, 128);
		char spi_flash_data_intro[] = "Flash DATA received: ";
		HAL_UART_Transmit(&huart1, (uint8_t*) spi_flash_data_intro, strlen(spi_flash_data_intro), 1000);
		HAL_UART_Transmit(&huart1, flashBufferRMCReceived, sizeof(flashBufferRMCReceived), 1000);
		//receiveTaxData();
		receiveRMCDataFromGPS();

		uart_transmit_string(&huart1,(uint8_t*) "\n\n");
		osDelay(1500);
  }
  /* USER CODE END StartSpiFlash */
}

