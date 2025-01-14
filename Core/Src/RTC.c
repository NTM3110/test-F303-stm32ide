/*
 * RTC.c
 *
 *  Created on: Nov 19, 2024
 *      Author: Admin
 */


/*
 * RTC.c
 *
 *  Created on: Nov 19, 2024
 *      Author: Admin
 */
#include "RTC.h"
#include "system_management.h"
#include "spi_flash.h"

void set_time (uint8_t hr, uint8_t min, uint8_t sec)
{
	RTC_TimeTypeDef sTime = {0};
	sTime.Hours = hr;
	sTime.Minutes = min;
	sTime.Seconds = sec;
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}
}

void set_date (uint8_t year, uint8_t month, uint8_t date)  // monday = 1
{
	RTC_DateTypeDef sDate = {0};
	sDate.Month = month;
	sDate.Date = date;
	sDate.Year = year;
	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}
}

void get_RTC_time_date(RMCSTRUCT *rmc)
{
	RTC_DateTypeDef gDate;
	RTC_TimeTypeDef gTime;

	  /* Get the RTC current Time */
	HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
	/* Get the RTC current Date */
	HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);

  /* Display time Format: hh:mm:ss */
  /* Format time as "HH:MM:SS" */
	printf("%02d:%02d:%02d\n", gTime.Hours, gTime.Minutes, gTime.Seconds);

	/* Format date as "YYYY-MM-DD" */
	printf("20%02d-%02d-%02d\n", gDate.Year, gDate.Month, gDate.Date);

	rmc->date.Yr = gDate.Year;
	rmc->date.Mon = gDate.Month;
	rmc->date.Day = gDate.Date;
	rmc->tim.hour = gTime.Hours;
	rmc->tim.min = gTime.Minutes;
	rmc->tim.sec = gTime.Seconds;

	//save_rmc_to_location_info(location_info);
//	snprintf((char*)output_buffer, 128, "Time to GMT+8 saved to RMC: 20%02d/%02d/%02d, %02d:%02d:%02d\n", rmc->date.Yr, rmc->date.Mon, rmc->date.Day, rmc->tim.hour, rmc->tim.min, rmc->tim.sec);
//	uart_transmit_string(&huart1, (uint8_t*) output_buffer);
}
