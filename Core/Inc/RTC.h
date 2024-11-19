/*
 * RTC.h
 *
 *  Created on: Nov 19, 2024
 *      Author: Admin
 */

#ifndef INC_RTC_H_
#define INC_RTC_H_


void set_time (uint8_t hr, uint8_t min, uint8_t sec);
void set_date (uint8_t year, uint8_t month, uint8_t date);
void get_RTC_time_date(RMCSTRUCT *rmc);


#endif /* INC_RTC_H_ */
