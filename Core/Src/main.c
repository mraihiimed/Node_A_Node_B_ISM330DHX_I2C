/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/

#include "main.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "vehiclefulectri.h"
#include "dht.h"
//#include "stm32f1xx_hal.h"
#include "Read_Function.h"
#include "ism330dhcx.h"
//#include "can.h"
//#include "tim.h"
//#include "gpio.h"
#include "systemclock.h"
//#include "i2c1.h"
//#include "uart.h"
//#include "peripherals_init.h"
#include "can_handler.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;
extern CAN_HandleTypeDef hcan;
float accel[3], gyro[3];


CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;

uint8_t RHI, RHD, TCI, TCD, SUM;
uint32_t pMillis, cMillis;
float tCelsius = 0;
float tFahrenheit = 0;
float RH = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  MotionRaw_T motion;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  Init_All_Peripherals();
  /* USER CODE BEGIN 2 */
  //Start UART peripheral
  uart_send_string("Start Debug: reached point X\r\n");
  // Start CAN peripheral
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    Error_Handler();
  }

  // Enable CAN RX interrupt
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }

  // Configure CAN TX Header
  TxHeader.DLC = 8;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.StdId = 0x321;

  // Load sample data
  TxData[0] = 0x11;
  TxData[1] = 0x22;
  TxData[2] = 0x33;
  TxData[3] = 0x44;
  TxData[4] = 0x55;
  TxData[5] = 0x66;
  TxData[6] = 0x77;
  TxData[7] = 0x88;
  // Init I2C1 via CubeMX or manually
  ISM330_Init();
  HAL_TIM_Base_Start(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(DHT11_Start())
	     {
	       RHI = DHT11_Read(); // Relative humidity integral
	       RHD = DHT11_Read(); // Relative humidity decimal
	       TCI = DHT11_Read(); // Celsius integral
	       TCD = DHT11_Read(); // Celsius decimal
	       SUM = DHT11_Read(); // Check sum
	       if (RHI + RHD + TCI + TCD == SUM)
	       {
	         // Can use RHI and TCI for any purposes if whole number only needed
	         tCelsius = (float)TCI + (float)(TCD/10.0);
	         tFahrenheit = tCelsius * 9/5 + 32;
	         RH = (float)RHI + (float)(RHD/10.0);
	         // Can use tCelsius, tFahrenheit and RH for any purposes
	         char msg[64];
	         snprintf(msg,sizeof(msg),"Temp:%.1f°C, Humidity: %.1f%%\r\n",tCelsius,RH);
	         uart_send_string(msg);
	       }
	       else
	       {
	    	   uart_send_string("DHT11 checksum error \r\n");
	       }
	       sendTemperatureHumidityMessage();
	     }
	    // Send CAN message
	    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	    {
	      Error_Handler();
	    }
	    uart_send_string("Debug: reached point X\r\n");
	    ISM330DHCX_ReadRaw(&motion);
	    sendMotionStatusToNodeA(&motion);
	    HAL_Delay(1000); // just idle loop
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
