#ifndef SPI_FLASH_H
#define SPI_FLASH_H

#include "main.h"
#include <stdint.h> // uint8_t
#include "cmsis_os.h"// HAL header files, SPI and GPIO defines
#include <time.h>

#include "Queue_GSM.h"


#define W25_CS_ENABLE()   HAL_GPIO_WritePin(SPI2_NCS_GPIO_Port, SPI2_NCS_Pin, GPIO_PIN_RESET)
#define W25_CS_DISABLE()  HAL_GPIO_WritePin(SPI2_NCS_GPIO_Port, SPI2_NCS_Pin, GPIO_PIN_SET)

#define W25_CMD_WRITE_ENABLE        0x06
#define W25_CMD_WRITE_DISABLE       0x04
#define W25_CMD_READ_STATUS_REG_1   0x05
#define W25_CMD_READ_JEDEC_ID       0x9f
#define W25_CMD_READ_UNIQUE_ID      0x4B
#define W25_CMD_READ_DATA           0x03
#define W25_CMD_PAGE_PROGRAM        0x02
#define W25_CMD_SECTOR_ERASE        0x20
#define W25_CMD_CHIP_ERASE          0x60

// Status Register 1 bits (see section 7.1 Status Registers)
#define W25_STATUS1_BUSY            1<<0
#define W25_STATUS1_WEL             1<<1
#define W25_STATUS1_BP0             1<<2
#define W25_STATUS1_BP1             1<<3
#define W25_STATUS1_BP2             1<<4
#define W25_STATUS1_TB              1<<5
#define W25_STATUS1_SEC             1<<6
#define W25_STATUS1_SRP0            1<<7

extern SPI_HandleTypeDef hspi1; // STM32 SPI instance
//extern uint16_t j,k,cnt,check;
//extern uint8_t gsvSentence[2048];
//extern uint8_t taxBuffer[128];

extern osMailQId tax_MailQId;
extern osMailQId RMC_MailQFLASHId;
extern osMailQId RMC_MailQGSMId;
extern osMailQId RMC_MailQLEDId;

extern osMailQId addr_MailQGSMId;
extern osMailQId uint8MailQueue;

#define TIMEOUT                 1000 // MS Timeout for HAL function calls
#define PAGE_PROGRAM_TIMEOUT    1000 // MS Timeout for Program
#define SECTOR_ERASE_TIMEOUT    1000 // MS Timeout for Sector Erase, Program, and Chip erase
#define CHIP_ERASE_TIMEOUT     	60000 // MS Timeout for Chip erase

#define W25_JEDEC_ID_BUF_SIZE   4   // bytes
#define W25_UNIQUE_ID_BUF_SIZE  8   // bytes
#define W25_PROGRAM_BLOCK_SIZE  256  // bytes - can write from 1 up to 256 bytes
#define W25_SECTOR_SIZE         4096 // bytes - used for erasing portions device
#define W25_DEVICE_SIZE         (16*1024*1024) // bytes (128MBit = 16MBytes)
#define W25_SECTOR_COUNT        (W25_DEVICE_SIZE/W25_SECTOR_SIZE)


#define SPI2_NCS_GPIO_Port			GPIOA
#define SPI2_NCS_Pin				GPIO_PIN_15

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

typedef struct RMCSTRUCT{
	TIME tim;
	DATE date;
	float speed;
	float course;
	int isValid;
	LOCATION lcation;
}RMCSTRUCT;


typedef struct GSM_MAIL_STRUCT{
	RMCSTRUCT rmc;
	uint32_t address;
}GSM_MAIL_STRUCT;

extern uint32_t result_address;
extern int is_flash_overflow;
extern volatile uint32_t start_addr_disconnect;
extern volatile uint32_t current_addr_gsm;
extern volatile uint32_t end_addr_disconnect;
extern int is_disconnect;
extern int is_using_flash;
extern Queue_GSM result_addr_queue;
extern Queue_GSM mail_sent_queue;

extern int mode;

int W25_ChipErase(void);

int W25_DelayWhileBusy(uint32_t msTimeout);

int W25_ReadJedecID();
int W25_ReadUniqueID(uint8_t *buf, int bufSize);
int W25_ReadMD(uint8_t *buf, int bufSize);
// Send Write Enable command, allowing writing to the device
int W25_WriteEnable(void );
void W25_Reset();
// Erase a 4096 byte sector
int W25_SectorErase(uint32_t address);

int W25_PageProgram(uint32_t address, uint8_t *buf, uint32_t count);

int W25_ReadData(uint32_t address, uint8_t *buf, int bufSize);

int checkSentToServer(uint32_t addr, Queue_GSM *queue);

void receiveRMCData(void);


#endif
