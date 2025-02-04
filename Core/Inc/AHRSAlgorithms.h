/*
 * AHRSAlgorithms.h
 *
 *  Created on: Jan 22, 2025
 *      Author: Admin
 */

#ifndef INC_AHRSALGORITHMS_H_
#define INC_AHRSALGORITHMS_H_

#define PI 3.14159

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
                              float gz, float mx, float my, float mz,
                              float deltat);
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
                            float gz, float mx, float my, float mz,
                            float deltat);
const float * getQ();



#endif /* INC_AHRSALGORITHMS_H_ */
