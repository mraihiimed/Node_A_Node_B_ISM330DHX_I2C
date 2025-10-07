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
#include <stdint.h>
#include <string.h>
#include "main.h"
#include "vehiclefulectri.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal.h"
#include "Read_Function.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ISM330_ADDR        0x6A << 1 // HAL expects 8-bit address (shifted)
#define ISM330_WHO_AM_I    0x0F
#define ISM330_WHO_AM_I_VAL 0x6B
#define ISM330_CTRL1_XL    0x10
#define ISM330_CTRL2_G     0x11
#define ISM330_OUTX_L_G    0x22
#define ISM330_OUTX_L_A    0x28

#define ACC_FS_2G       0.061f   // mg/LSB → g conversion
#define GYRO_FS_250DPS  8.75f    // mdps/LSB → dps conversion
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
float accel[3], gyro[3];
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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

// ---------- Read Accel + Gyro ----------
void ISM330_Read_AccelGyro(int16_t *accel, int16_t *gyro)
{
    uint8_t buffer[12];

    // Auto-increment read from OUTX_L_G (0x22)
    HAL_I2C_Mem_Read(&hi2c1, ISM330_ADDR, ISM330_OUTX_L_G, I2C_MEMADD_SIZE_8BIT, buffer, 12, HAL_MAX_DELAY);

    // Gyroscope
    gyro[0] = (int16_t)(buffer[1] << 8 | buffer[0]);
    gyro[1] = (int16_t)(buffer[3] << 8 | buffer[2]);
    gyro[2] = (int16_t)(buffer[5] << 8 | buffer[4]);

    // Accelerometer
    accel[0] = (int16_t)(buffer[7] << 8 | buffer[6]);
    accel[1] = (int16_t)(buffer[9] << 8 | buffer[8]);
    accel[2] = (int16_t)(buffer[11] << 8 | buffer[10]);
}
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
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
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
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {



//       //ISM330_Read_AccelGyro(accel, gyro);
//		ISM330_Read_AccelGyro_g_dps(accel, gyro);
//	    // Send CAN message
//	    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
//	    {
//	      Error_Handler();
//	    }
//
//
//	    // Blink LED on PC13
//	    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//	    // Wait 1 second
//	    HAL_Delay(1000); // just idle loop


	    ISM330DHCX_ReadRaw(&motion);
	    sendMotionStatusToNodeA(&motion);
	    HAL_Delay(1000); // just idle loop
  }
  /* USER CODE END 3 */
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);

    handleCANRxMessage(RxHeader.StdId, RxData, RxHeader.DLC);

    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);  // Blink LED when message received
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}
/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  CAN_FilterTypeDef filterConfig;
  filterConfig.FilterActivation = ENABLE;
  filterConfig.FilterBank = 0;
  filterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filterConfig.FilterIdHigh = 0x0000;
  filterConfig.FilterIdLow = 0x0000;
  filterConfig.FilterMaskIdHigh = 0x0000;
  filterConfig.FilterMaskIdLow = 0x0000;
  filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

  HAL_CAN_ConfigFilter(&hcan, &filterConfig);

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void sendMotionStatusToNodeA(const MotionRaw_T* motion)
{
    CAN_TxHeaderTypeDef txHeader;
    uint8_t txData[8];
    uint32_t txMailbox;

    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 8;

    // Frame 1: ax, ay, az
    txHeader.StdId = 0x400;
    memcpy(txData, &motion->ax, 6); // ax, ay, az
    txData[6] = 0xAA; // marker or padding
    txData[7] = 0x55;
    HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);

    // Frame 2: gx, gy, gz
    txHeader.StdId = 0x401;
    memcpy(txData, &motion->gx, 6); // gx, gy, gz
    txData[6] = 0xCC;
    txData[7] = 0x33;
    HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);
}

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
