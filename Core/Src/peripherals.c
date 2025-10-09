/*
 * peripherals.c
 *
 *  Created on: Oct 9, 2025
 *      Author: Geek
 */


#include "stm32f1xx_hal.h"

// Define all shared peripheral handles
CAN_HandleTypeDef hcan;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart2;
