/*
 * ICM-20948.h
 *
 *  Created on: Jan 18, 2025
 *      Author: Admin
 */

#ifndef INC_ICM_20948_H_
#define INC_ICM_20948_H_

#include "time.h"
#include "main.h"
#include "cmsis_os.h"

uint16_t accel_data[3];
uint16_t gyro_data[3];
int16_t mag_data[3];

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;

#define I2C_BUS								&hi2c1
#define UART_BUS							&huart1

//TODO: MASK THE REGISTER ADDRESS with 0xFF (&0xFF) to make it in 8 bit range in case of exceeding range.
#define ICM_ADDRESS 						(0b1101000 << 1)// The LSB of the SLAVE ADDRESS is defined by AD0 bit from MODULE. SET DEFAULT TO 0.
#define ACCEL_GYRO_REG_START_ADDR			(0x2D << 1)
#define USER_BANK_SEL						(0x7F << 1)


#define USER_BANK_0							0x00
#define USER_BANK_1							0x10
#define USER_BANK_2							0x20
#define USER_BANK_3							(uint8_t)0x30


#define USER_CTRL_R 						0x03
#define WHO_AM_I_R							0x00

#define PWR_MGMT_1 							0x06
#define PWR_MGMT_2							0x07
#define GYRO_CONFIG_1						0x01


#define CLK_BEST_AVAIL						0x01
#define GYRO_RATE_250						0x00
#define GYRO_LPF_17HZ 						0x29


#define MAG_I2C_ADDR						0x0C
#define MAG
#define I2C_SLAVE_SELECT					0x03
#define I2C_REGISTER_SELECT					0x04
#define I2C_DATA_ASSIGN						0x06

void ICM_PowerOn();
uint8_t ICM_WHOAMI(void);
void ICM_SelectBank(uint8_t bank);
void ICM_ReadAccelGyroData(void);
void ICM_ReadMagData(int16_t magn[3]);
uint16_t ICM_Initialize(void);
void ICM_SelectBank(uint8_t bank);
void ICM_Enable_I2C(void);
void ICM_SetClock(uint8_t clk);
void ICM_AccelGyroOff(void);
void ICM_AccelGyroOn(void);
void ICM_SetGyroRateLPF(uint8_t rate, uint8_t lpf);
void ICM_SetGyroLPF(uint8_t lpf);



#endif /* INC_ICM_20948_H_ */
