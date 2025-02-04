/*
 * ICM-20948.c
 *
 *  Created on: Jan 18, 2025
 *      Author: Admin
 */
#include "ICM-20948.h"
#include "AHRSAlgorithms.h"

#include "cmsis_os.h"
#include "main.h"
#include <stdio.h>

float accel_X = 0.0, accel_Y = 0.0, accel_Z = 0.0;
float gyro_X = 0.0, gyro_Y = 0.0, gyro_Z = 0.0;
float mag_X = 0.0, mag_Y = 0.0, mag_Z = 0.0;

void ICM_ReadBytes(uint16_t reg, uint8_t *pData, uint16_t Size){
	int status = HAL_I2C_Mem_Read(&hi2c1, ICM_ADDRESS, reg, 1, pData, Size, 1000);
	if(status == HAL_OK){
		printf("ICM READ BYTES SUCCESSFULLY: %s", pData);
	}
	else{
		printf("ICM READ BYTES FAILED:status %d", status);
	}
	//TODO:Adjust the delay to be reasonable
}

void ICM_WriteBytes(uint16_t reg, uint8_t *pData, uint16_t Size){

	int status = HAL_I2C_Mem_Write(&hi2c1, ICM_ADDRESS, reg, 1, pData, Size, 1000);
	if(status == HAL_OK){
		printf("ICM WRITE BYTES SUCCESSFULLY: %s", pData);
	}
	else{
		printf("ICM WRITE BYTES FAILED:status %d", status);
	}
	//TODO:Adjust the delay to be reasonable
}

void ICM_ReadOneByte(uint8_t reg, uint8_t* pData) // ***
{
	int status = HAL_I2C_Mem_Read(&hi2c1, ICM_ADDRESS, reg, 1, pData, 1, 1000);
	if(status == HAL_OK){
		printf("ICM READ ONE BYTE SUCCESSFULLY: %s", pData);
	}
	else{
		printf("ICM READ ONE BYTE FAILED:status %d", status);
	}
	//TODO:Adjust the delay to be reasonable
}
void ICM_WriteOneByte(uint8_t reg, uint8_t pData)
{
	int status = HAL_I2C_Mem_Write(&hi2c1, ICM_ADDRESS, reg, 1, &pData, 1, 1000);
	if(status == HAL_OK){
		printf("ICM WRITE ONE BYTE SUCCESSFULLY: %02x", pData);
	}
	else{
		printf("ICM WRITE ONE BYTE FAILED: status %d", status);
	}
	//TODO:Adjust the delay to be reasonable
}

/*
 *
 * AUX I2C abstraction for magnetometer
 *
 */
void I2C_Mag_Write(uint8_t reg,uint8_t value)
{
  	ICM_WriteOneByte(USER_BANK_SEL, USER_BANK_3);

  	osDelay(1);
  	ICM_WriteOneByte(I2C_SLAVE_SELECT, MAG_I2C_ADDR);//mode: write

  	osDelay(1);
  	ICM_WriteOneByte(I2C_REGISTER_SELECT, reg);//set reg addr

  	osDelay(1);
  	ICM_WriteOneByte(I2C_DATA_ASSIGN, value);//send value

  	osDelay(1);
}

static uint8_t I2C_Mag_Read(uint8_t reg)
{
	uint8_t  Data;
	ICM_WriteOneByte(USER_BANK_SEL, 0x30);
	osDelay(1);
	ICM_WriteOneByte(I2C_SLAVE_SELECT, 0x0C|0x80);
	osDelay(1);
	ICM_WriteOneByte(I2C_REGISTER_SELECT, reg);// set reg addr
	osDelay(1);
	ICM_WriteOneByte(I2C_DATA_ASSIGN, 0xff);//read
	osDelay(1);

	ICM_WriteOneByte(USER_BANK_SEL, 0x00);
	ICM_ReadOneByte(0x3B,&Data);
	osDelay(1);
	return Data;
}

/*
 *
 * Read magnetometer
 *
 */
void ICM_ReadMagData(int16_t magn[3]) {
	uint8_t mag_buffer[10];

	//WHO I AM
	mag_buffer[0] = I2C_Mag_Read(0x01);
	printf("WHO AM I AK09916: %s", mag_buffer);

	//X-axis
	mag_buffer[1] = I2C_Mag_Read(0x11);
	mag_buffer[2] = I2C_Mag_Read(0x12);
	magn[0]= mag_buffer[1]|mag_buffer[2]<<8;

	mag_X = magn[0] * 0.15;

	//Y-axis
	mag_buffer[3] = I2C_Mag_Read(0x13);
	mag_buffer[4] = I2C_Mag_Read(0x14);
	magn[1]=mag_buffer[3]|mag_buffer[4]<<8;

	mag_Y = magn[1] * 0.15;

	//Z-axis
	mag_buffer[5] = I2C_Mag_Read(0x15);
	mag_buffer[6] = I2C_Mag_Read(0x16);
	magn[2]=mag_buffer[5]|mag_buffer[6]<<8;

	mag_Z = magn[2] * 0.15;

	I2C_Mag_Write(0x31,0x01);
}

void ICM_SelectBank(uint8_t bank) {
	ICM_WriteOneByte(USER_BANK_SEL, bank);  //The value of bank is 0x10 or 0x20 or 0x30 bc it starts at 4th bit
}

void ICM_Enable_I2C(void){
	ICM_WriteOneByte(USER_CTRL_R, 0x68);
}

uint8_t ICM_WHOAMI(void){
	uint8_t check = 0x01;
	ICM_ReadOneByte(WHO_AM_I_R, &check);
	return check;
}

void ICM_SetClock(uint8_t clk){
	ICM_WriteOneByte(PWR_MGMT_1, clk);
}

void ICM_AccelGyroOff(void) {
	ICM_WriteOneByte(PWR_MGMT_2, (0x38 | 0x07));
}

void ICM_AccelGyroOn(void) {
	ICM_WriteOneByte(PWR_MGMT_2, (0x00 | 0x00));
}

void ICM_SetGyroRateLPF(uint8_t rate, uint8_t lpf){
	ICM_WriteOneByte(GYRO_CONFIG_1, (rate|lpf)); //
}

void ICM_ReadAccelGyroData(void) {
	uint8_t raw_data[12];
	ICM_ReadBytes(0x2D, raw_data, 12);

	accel_data[0] = (raw_data[0] << 8) | raw_data[1];
	accel_data[1] = (raw_data[2] << 8) | raw_data[3];
	accel_data[2] = (raw_data[4] << 8) | raw_data[5];

	gyro_data[0] = (raw_data[6] << 8) | raw_data[7];
	gyro_data[1] = (raw_data[8] << 8) | raw_data[9];
	gyro_data[2] = (raw_data[10] << 8) | raw_data[11];

//	accel_data[0] = accel_data[0] / 8;
//	accel_data[1] = accel_data[1] / 8;
//	accel_data[2] = accel_data[2] / 8;

	accel_X = accel_data[0] / 4096;
	accel_Y = accel_data[1] / 4096;
	accel_Z = accel_data[2] / 4096;

//	gyro_data[0] = gyro_data[0] / 250;
//	gyro_data[1] = gyro_data[1] / 250;
//	gyro_data[2] = gyro_data[2] / 250;

	gyro_X = gyro_data[0] / 131;
	gyro_Y = gyro_data[1] / 131;
	gyro_Z = gyro_data[2] / 131;



}

uint16_t ICM_Initialize(void) {
	ICM_SelectBank(USER_BANK_2);
	osDelay(20);

	ICM_SetGyroRateLPF(GYRO_RATE_250, GYRO_LPF_17HZ);
	osDelay(10);
	// Set gyroscope sample rate to 100hz (0x0A) in GYRO_SMPLRT_DIV register (0x00)
	ICM_WriteOneByte(0x00, 0x0A);
	osDelay(10);

	// Set accelerometer low pass filter to 136hz (0x11) and the rate to 8G (0x04) in register ACCEL_CONFIG (0x14)
	ICM_WriteOneByte(0x14, (0x04 | 0x11));

	// Set accelerometer sample rate to 225hz (0x00) in ACCEL_SMPLRT_DIV_1 register (0x10)
	ICM_WriteOneByte(0x10, 0x00);
	osDelay(10);

	// Set accelerometer sample rate to 100 hz (0x0A) in ACCEL_SMPLRT_DIV_2 register (0x11)
	ICM_WriteOneByte(0x11, 0x0A);
	osDelay(10);

	ICM_SelectBank(USER_BANK_2);
	osDelay(20);

	/*
	 *
	 *
	 *
	 * Configure AUX_I2C Magnetometer(onboard ICM-20948)
	 *
	 *
	 */
	ICM_SelectBank(USER_BANK_0);
	// INT_PIN_CFG
	ICM_WriteOneByte(0x0F, 0x30);

	//USER_CTRL: Enable I2C Master
	ICM_WriteOneByte(0x03, 0x20);

	ICM_SelectBank(USER_BANK_3);

	//I2C_MST_CTRL
	//TODO: Maybe set back to 0x07: For clock frequency of MST I2c
	ICM_WriteOneByte(0x01, 0x4D);
	//I2C_MST_DELAY_CTRL
	ICM_WriteOneByte(0x02, 0x01);

	//I2C_SLV0_CTRL
	ICM_WriteOneByte(0x05, 0x81);
	I2C_Mag_Write(0x32, 0x01);
	osDelay(1000);
	I2C_Mag_Write(0x31, 0x02);
	return 1337;
}

void ICM_PowerOn(){
	uint8_t who_am_i_value = 0xEA;
	uint8_t test = 0x01;
	test = ICM_WHOAMI();
	if(test == who_am_i_value){
		printf("WHO AM I successfully");
		ICM_SelectBank(USER_BANK_0);
		osDelay(10);
		ICM_Enable_I2C();
		osDelay(10);
		ICM_SetClock((uint8_t)CLK_BEST_AVAIL);
		osDelay(10);
		ICM_AccelGyroOff();
		osDelay(20);
		ICM_AccelGyroOn();
		osDelay(10);
		ICM_Initialize();
	}
	else{
		printf("WHO AM I failed: 0x%02x", test);
	}
}


void StartICM(){
	printf("Starting ICM task");
	ICM_SelectBank(USER_BANK_0);
	osDelay(10);
	ICM_PowerOn();
	osDelay(10);

	for(;;){
		ICM_SelectBank(USER_BANK_0);
		osDelay(10);
		ICM_ReadAccelGyroData();

		uint16_t mag_data[3];
		ICM_ReadMagData(mag_data);
		printf("\n\n--------------- RAW DATA -----------------\n");
		printf(" (Ax: %u | Ay: %u | Az: %u)   \n"
				"(Gx: %u | Gy: %u | Gz: %u)   \n"
				"(Mx: %i | My: %i | Mz: %i)	  \n"
				" \r\n",
				accel_data[0],accel_data[1],accel_data[2],
				gyro_data[0], gyro_data[1], gyro_data[2],
				mag_data[0], mag_data[1], mag_data[2]);

		//Sensitivity of MAGNE is fixed at 0.15uT/LSB
		mag_X = 0.15 * mag_data[0];
		mag_Y = 0.15 * mag_data[1];
		mag_Z = 0.15 * mag_data[2];

		printf("\n\n--------------- CALCULATED DATA -----------------\n");
		printf(" (Ax: %.02f | Ay: %.02f | Az: %.02f)   \n"
				"(Gx: %.02f | Gy: %.02f | Gz: %.02f)   \n"
				"(Mx: %.02f | My: %.02f | Mz: %.02f)	\n"
				" \r\n",
				accel_X,accel_Y,accel_Z,
				gyro_X, gyro_Y, gyro_Z,
				mag_X, mag_Y, mag_Z);

		osDelay(200);
	}

	//TODO: SAVE DATA TO AN ARRAY TO CALIBRATION.
}

