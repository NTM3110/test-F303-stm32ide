#ifndef GSM_H
#define GSM_H

#include "main.h"
#include "RS232-UART1.h"
#include "Queue_GSM.h"

#define SIM_RESPONSE_MAX_SIZE						512
#define SIM_GPIO_Port								GPIOC
#define SIM_GPIO_PWR_CTRL_Pin						GPIO_PIN_0

#define APN_NAME									"e-connect"
#define APN_USERNAME								""
#define APN_PASSWD									""
#define APN_AUTHEN									1
#define SERVICE_TYPE 								"TCP"
#define IP_ADDRESS									"142.4.205.26"
#define REMOTE_PORT									5015
#define CHECK_RESPONSE								"OK"
// JT808 MESSAGE STRUCT
#define PLATE_NO_LEN 20
typedef struct {
    uint8_t start_mask;
    uint8_t message_type[2];
    uint8_t message_length[2];
    uint8_t terminal_phone_number[6];
    uint8_t message_serial_number[2];
    uint8_t province_ID[2];
    uint8_t city_ID[2];
    uint8_t manufacturer_ID[5];
    uint8_t terminal_type[8];
    uint8_t terminal_ID[7];
    uint8_t plate_color;
    uint8_t plate_no[PLATE_NO_LEN];
    uint8_t check_sum;
    uint8_t end_mask;
} JT808_TerminalRegistration;

typedef struct {
    uint8_t start_mask;                    // 1 byte
    uint8_t message_type[2];               // 2 bytes
    uint8_t message_length[2];             // 2 bytes
    uint8_t terminal_phone_number[6];     // 6 bytes
    uint8_t terminal_serial_number[2];      //2 bytes
    uint8_t alarm[4];                      // 4 bytes
    uint8_t status[4];                     // 4 bytes
    uint8_t latitude[4];                   // 4 bytes (32-bit integer)
    uint8_t longitude[4];                  // 4 bytes (32-bit integer)
    uint8_t altitude[2];                   // 2 bytes (16-bit integer)
    uint8_t speed[2];                      // 2 bytes
    uint8_t direction[2];                  // 2 bytes
    uint8_t timestamp[6];                  // 6 bytes
    uint8_t mileage[6];                    // 6 bytes
    uint8_t oil[2];                      // 2 bytes
    uint8_t driving_record_speed[2];         // 2 bytes
    uint8_t vehicle_id[3];                 // 3 bytes
    uint8_t signal[1];
    uint8_t additional[9];                  // 9 bytes
    uint8_t end_mask;                      // 1 byte
} JT808_LocationInfoReport;


//COMMAND AT
#define FIRST_CHECK									"AT\r\n"
#define CHECK_SIM_READY								"AT+CPIN?\r\n"
#define GET_SIM_CCID								"AT+QCCID\r\n"
#define GET_IMEI									"AT+CGSN=1\r\n"
#define GET_MODEL_IDENTI							"AT+CGMM\r\n"
#define CONFIGURE_CS_SERVICE						"AT+CREG=2\r\n"
#define CONFIGURE_PS_SERVICE						"AT+CGREG=2\r\n"
#define CHECK_SIGNAL_QUALITY						"AT+CSQ\r\n"
#define CHECK_CONFIGURE_APN							"AT+QICSGP=1\r\n"
#define CHECK_ACTIVATE_CONTEXT						"AT+QIACT?\r\n"
#define CONFIG_URC_PORT								"AT+QURCCFG=\"urcport\",\"uart1\""


extern RTC_HandleTypeDef hrtc;

#define SIM_ENABLE()   HAL_GPIO_WritePin(SIM_GPIO_Port, SIM_GPIO_PWR_CTRL_Pin, GPIO_PIN_SET)
#define SIM_DISABLE()  HAL_GPIO_WritePin(SIM_GPIO_Port, SIM_GPIO_PWR_CTRL_Pin	, GPIO_PIN_RESET)


extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim3;

extern osThreadId GSMHandle;

//Queue_GSM mail_sent_queue;


void send_AT_command(const char *command);

void init_SIM_module();

int configure_APN(int context_id);

int activate_context(int context_id);

void check_activate_context();

int open_socket_service(int context_id, int connect_id, int local_port, int access_mode);

void SIM_UART_ReInitializeRxDMA(void);

void StartGSM(void const * argument);
#endif
