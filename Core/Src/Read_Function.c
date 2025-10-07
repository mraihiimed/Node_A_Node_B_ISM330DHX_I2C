/*
 * Read_Function.c
 *
 *  Created on: Sep 24, 2025
 *      Author: Geek
 */


#include <stdint.h>
#include "Read_Function.h"
#include "main.h"
#define ACC_FS_2G       0.061f   // mg/LSB → g conversion
#define GYRO_FS_250DPS  8.75f    // mdps/LSB → dps conversion



// Implementation
void ISM330_Read_AccelGyro_g_dps(float accel_g[3], float gyro_dps[3])
{
    int16_t raw_accel[3], raw_gyro[3];

    // Read raw sensor data
    ISM330_Read_AccelGyro(raw_accel, raw_gyro);

    // Convert accelerometer
    for (int i = 0; i < 3; i++)
        accel_g[i] = (float)(raw_accel[i] * ACC_FS_2G / 1000.0f); // mg → g

    // Convert gyroscope
    for (int i = 0; i < 3; i++)
        gyro_dps[i] = (float)(raw_gyro[i] * GYRO_FS_250DPS / 1000.0f); // mdps → dps
}
