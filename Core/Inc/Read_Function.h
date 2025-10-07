/*
 * Read_Function.h
 *
 *  Created on: Sep 24, 2025
 *      Author: Geek
 */

#ifndef INC_READ_FUNCTION_H_
#define INC_READ_FUNCTION_H_
#include "main.h"
// Function prototype
void ISM330_Read_AccelGyro_g_dps(float accel_g[3], float gyro_dps[3]);
extern void ISM330_Read_AccelGyro(int16_t *accel, int16_t *gyro);
#endif /* INC_READ_FUNCTION_H_ */
