/*
 * ism330dhcx.c
 *
 *  Created on: Oct 9, 2025
 *      Author: Geek
 */
#include "main.h"
#include "ism330dhcx.h"

// ---------- Read Accel + Gyro ----------
extern I2C_HandleTypeDef hi2c1; // or whichever I2C instance you're using

void ISM330DHCX_ReadRaw(MotionRaw_T* motion)
{
    uint8_t rawData[12]; // 6 bytes accel + 6 bytes gyro

    // Read accelerometer data
    HAL_I2C_Mem_Read(&hi2c1, ISM330_ADDR, ISM330_OUTX_L_A, I2C_MEMADD_SIZE_8BIT, &rawData[0], 6, HAL_MAX_DELAY);

    // Read gyroscope data
    HAL_I2C_Mem_Read(&hi2c1, ISM330_ADDR, ISM330_OUTX_L_G, I2C_MEMADD_SIZE_8BIT, &rawData[6], 6, HAL_MAX_DELAY);

    // Convert LSB to int16_t
    motion->ax = (int16_t)(rawData[1] << 8 | rawData[0]);
    motion->ay = (int16_t)(rawData[3] << 8 | rawData[2]);
    motion->az = (int16_t)(rawData[5] << 8 | rawData[4]);

    motion->gx = (int16_t)(rawData[7] << 8 | rawData[6]);
    motion->gy = (int16_t)(rawData[9] << 8 | rawData[8]);
    motion->gz = (int16_t)(rawData[11] << 8 | rawData[10]);
}
// ---------- I2C Read/Write ----------
uint8_t ISM330_I2C_Read(uint8_t reg)
{
    uint8_t value;
    HAL_I2C_Mem_Read(&hi2c1, ISM330_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
    return value;
}

void ISM330_I2C_Write(uint8_t reg, uint8_t value)
{
    HAL_I2C_Mem_Write(&hi2c1, ISM330_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
}

// ---------- Init Sensor ----------
void ISM330_Init(void)
{
    uint8_t whoami = ISM330_I2C_Read(ISM330_WHO_AM_I);
    if (whoami != ISM330_WHO_AM_I_VAL) while(1); // error

    // Accelerometer: 104 Hz, ±2g
    ISM330_I2C_Write(ISM330_CTRL1_XL, 0x40);

    // Gyroscope: 104 Hz, 250 dps
    ISM330_I2C_Write(ISM330_CTRL2_G, 0x40);
}
