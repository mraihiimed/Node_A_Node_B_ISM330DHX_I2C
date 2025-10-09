/*
 * can.c
 *
 *  Created on: Oct 9, 2025
 *      Author: Geek
 */
#include "main.h"
#include "can.h"
#include "stm32f1xx_hal.h"
/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
extern CAN_HandleTypeDef hcan;

void MX_CAN_Init(void)
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

