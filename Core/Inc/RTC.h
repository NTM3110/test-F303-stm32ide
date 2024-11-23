/*
 * RTC.h
 *
 *  Created on: Nov 19, 2024
 *      Author: Admin
 */

#ifndef RTC_H
#define RTC_H

#include "string.h"
#include "cmsis_os.h"
#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

typedef struct RMCSTRUCT RMCSTRUCT;

extern RTC_HandleTypeDef hrtc;
extern UART_HandleTypeDef huart1;

void set_time (uint8_t hr, uint8_t min, uint8_t sec);
void set_date (uint8_t year, uint8_t month, uint8_t date);
void get_RTC_time_date(RMCSTRUCT *rmc);


#endif /* INC_RTC_H_ */
