/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal.h"
#include "peripherals_init.h"
#include "can.h"
#include "uart.h"
#include "tim.h"
#include "gpio.h"
#include "i2c1.h"
#include "motion_types.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define ISM330_ADDR        0x6A << 1 // HAL expects 8-bit address (shifted)
#define ISM330_WHO_AM_I    0x0F
#define ISM330_WHO_AM_I_VAL 0x6B
#define ISM330_CTRL1_XL    0x10
#define ISM330_CTRL2_G     0x11
#define ISM330_OUTX_L_G    0x22
#define ISM330_OUTX_L_A    0x28

#define ACC_FS_2G       0.061f   // mg/LSB → g conversion
#define GYRO_FS_250DPS  8.75f    // mdps/LSB → dps conversion
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void ISM330DHCX_ReadRaw(MotionRaw_T* motion);
void sendMotionStatusToNodeA(const MotionRaw_T* motion);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
void uart_send_char(char c);
char uart_receive_char(void);
void uart_send_string(const char *str);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
