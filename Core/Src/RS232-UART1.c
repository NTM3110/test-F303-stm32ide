#include "RS232-UART1.h"
#include "cmsis_os.h"
#include "string.h"
#include "stdint.h"
#include <stdio.h>
#include "stdlib.h"
#include "math.h"
#include <inttypes.h>

#include "system_management.h"
#include "spi_flash.h"


uint16_t j,k,cnt,check;
RingBufferDmaU8_TypeDef rs232Ext2RxDMARing;
extern osMailQId tax_MailQId;

uint8_t taxBuffer[128];
uint8_t gsvSentence[2048];
TAX_MAIL_STRUCT taxMailStruct;
int is_saving_debug = 0;
uint8_t current_debug_page[256];
volatile uint32_t current_addr_rs232 = 0x7000;

uint8_t start_cmd[] = "START SAVE DEBUG!";
uint8_t end_cmd[] = "END_SAVE_DEBUG!";



int is_get_cmd = 0;
int is_get_start_cmd = 0;
int is_get_end_cmd = 0;

// RMCSTRUCT rmc;
// #define GMT 		000

// int isRMCExist = 0;
// int inx = 0;
// int hr=0,min=0,day=0,mon=0,yr=0;
// int daychange = 0;

uint8_t message1[] = "Hello from DSS\n";

void RingBufferDmaU8_initUSARTRx(RingBufferDmaU8_TypeDef* ring, UART_HandleTypeDef* husart, uint8_t* buffer, uint16_t size) // cai dat dma
{
  ring->buffer = buffer;
  ring->size = size;
  ring->tailPtr = buffer;
  ring->dmaHandle = husart->hdmarx;
  HAL_UART_Receive_DMA(husart, buffer, size);
}


#ifdef USING_CCMRAM
__attribute__((section("ccmram")))
#endif
uint16_t RingBufferDmaU8_available(RingBufferDmaU8_TypeDef* ring) // kiem tra trang thai su dung dma
{
#ifdef __HAL_DMA_GET_COUNTER
  uint32_t leftToTransfer = __HAL_DMA_GET_COUNTER(ring->dmaHandle);
#else
  uint32_t leftToTransfer = ring->dmaHandle->Instance->CNDTR;
#endif
  volatile uint8_t const* head = ring->buffer + ring->size - leftToTransfer;
  volatile uint8_t const* tail = ring->tailPtr;
  if (head >= tail) {
    return head - tail;
  } else {
    return head - tail + ring->size;
  }
}


void rs232Ext2_InitializeRxDMA(void)// ham khoi tao lai DMA
{
	HAL_StatusTypeDef ret = HAL_UART_Abort(&huart1);
	if(ret != HAL_OK)
	{
		Error_Handler();			
	}		
	HAL_Delay(50);	//	50 is OK
	//memset(gnssDmaRingBufferMemory, 0x20, sizeof(gnssDmaRingBufferMemory));	// insert buffer with space character	
	RingBufferDmaU8_initUSARTRx(&rs232Ext2RxDMARing, &huart1, gsvSentence, READLOG_BLOCK_BUFFER_LENGHT);
}

void sendTaxData(uint8_t *arrayData, uint32_t size) {
	TAX_MAIL_STRUCT *mail = (TAX_MAIL_STRUCT *)osMailAlloc(tax_MailQId, osWaitForever); // Allocate mail
	if (mail != NULL) {
		// Copy data into the mail structure (up to ARRAY_SIZE)
		for (int i = 0; i < size ; i++) {
			mail->data[i] = arrayData[i];
		}
		osMailPut(tax_MailQId, mail); // Put mail in queue
	}
}

void Bill_Decode(){
	for (uint16_t i = 0; i < READLOG_BLOCK_BUFFER_LENGHT-1; i++)
	{
		if ((gsvSentence[i] == ')') & (gsvSentence[i+1]==':'))
		{
			check++;
		}	
	}
	if (check > 7)
	{	
		check=0;
		for (uint16_t i = 0; i < READLOG_BLOCK_BUFFER_LENGHT; i++) 
		{	
			if ( (gsvSentence[i] == ':') & (gsvSentence[i-1] == 'E') )
			{
					for (j=0; j < 10; j++)
					{
							if((gsvSentence[j+i+1] > 47) & (gsvSentence[j+i+1] < 123))
							{	
								k++;
								taxBuffer[k]=gsvSentence[j+i+1];
								cnt++;
							}	
					}	
							k++;
							taxBuffer[k]=';';	
							i=i+cnt;				
			}
			if ((gsvSentence[i] == ':') & (gsvSentence[i-1] == 'i'))
			{
					for (j=0; j < 5; j++)
					{
							if((gsvSentence[j+i+1] > 47) & (gsvSentence[j+i+1] < 58 ))
							{	
								k++;
								taxBuffer[k]=gsvSentence[j+i+1];
								cnt++;
							}	
					}	
							k++;
							taxBuffer[k]=';';	
							i=READLOG_BLOCK_BUFFER_LENGHT;
			}		
		}
		
		for (uint16_t i = 0; i < READLOG_BLOCK_BUFFER_LENGHT; i++) 
		{
			if ( (gsvSentence[i] == ')') & (gsvSentence[i+1]==':') )
			{
				check++;
				if (check<=8)
				{	
					for (j=0; j < 30; j++)
					{
						if(gsvSentence[j+i+2]!=0x0A)	
						{	
							if( ( (gsvSentence[j+i+2] > 47) & (gsvSentence[j+i+2] < 58)) ||(gsvSentence[j+i+2] == 44 ) )
							{	
								k++;
								taxBuffer[k]=gsvSentence[j+i+2];
								cnt++;
							}	

						}	
						else
						{
							j=READLOG_BLOCK_BUFFER_LENGHT;
							i=i+cnt;
							cnt=0;
							k++;
							taxBuffer[k]=';';
						}
					 }	
				}	
			}
		}
		k++;
		taxBuffer[k]='#';
		for (j=0;j<110-k-1;j++)
		{
			taxBuffer[j+k+1]=0x00;
		}
		
		char tax_buffer_received_intro[] = "Tax Buffer received UART: ";
		HAL_UART_Transmit(&huart1, (uint8_t*) tax_buffer_received_intro, strlen(tax_buffer_received_intro), 1000);
		HAL_UART_Transmit(&huart1, taxBuffer, sizeof(taxBuffer), 100);
		HAL_UART_Transmit(&huart1, (uint8_t*)"\n", 1, 100);
		sendTaxData(taxBuffer,128);
		check = 0;
		j = 1;
		k=0;
		cnt=0;
		memset(taxBuffer, 0x00, 128);
		memset(gsvSentence, 0x00, 2048);
		rs232Ext2_InitializeRxDMA();
	}
}
void handleCmd(uint8_t *buffer){
    // Check for start_cmd
    if (strstr((char*) buffer, (char*)start_cmd)) {
        printf("Start command found! Handling START SAVE DEBUG logic...\n");
        is_saving_debug = 1;
        W25_Reset();
        W25_SectorErase(DEBUG_START_ADDRESS);
        // Handle START SAVE DEBUG logic here
    }

    // Check for end_cmd
    if (strstr((char*) buffer, (char*)end_cmd)) {
        printf("End command found! Handling END_SAVE_DEBUG logic...\n");
        while(current_addr_rs232 < 0x8000){
        	printf("SHOW ALL THE DEBUG SAVED IN FLASH: UNTIL");
        	W25_Reset();
        	W25_ReadData(current_addr_rs232, current_debug_page, 256);
        	printf((char*)current_debug_page);
        	printf("\n");
        	current_addr_rs232 += 256;
        	osDelay(200);
        }
        is_saving_debug = 0;
        // Handle END_SAVE_DEBUG logic here
    }

}

void getCmd(){
	for (uint16_t i = 0; i < READLOG_BLOCK_BUFFER_LENGHT-1; i++)
	{
		if (gsvSentence[i] == '!')
		{
			is_get_cmd = 1;
		}
	}
	if(is_get_cmd == 1){
		printf((char*)gsvSentence);
		printf("\n\n");
		handleCmd(gsvSentence);
		memset(gsvSentence, 0x00, 2048);
		rs232Ext2_InitializeRxDMA();
		is_get_cmd = 0;
	}
}


void StartUART1(void const * argument)
{
//	tax_MailQId = osMessageQueueNew(5, sizeof(TAX_MAIL_STRUCT), NULL);
	HAL_UART_Transmit(&huart1, (uint8_t*) "STARTING UART1", strlen("STARTING UART1"), 1000);
  /* USER CODE BEGIN StartUART1 */
	RingBufferDmaU8_initUSARTRx(&rs232Ext2RxDMARing, &huart1, gsvSentence, READLOG_BLOCK_BUFFER_LENGHT);
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	osDelay(500);
	//W25_Reset();
	HAL_UART_Transmit(&huart1, message1, sizeof(message1), 100);
//	Bill_Decode();
	getCmd();

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	osDelay(500);
  }
  /* USER CODE END StartUART1 */
}
