/*
 * can_handler.c
 *
 *  Created on: Oct 9, 2025
 *      Author: Geek
 */
#include "can_handler.h"
#include "can.h"
#include "main.h"
#include "vehiclefulectri.h"
#include <string.h>

extern CAN_HandleTypeDef hcan;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);

    handleCANRxMessage(RxHeader.StdId, RxData, RxHeader.DLC);

    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);  // Blink LED when message received
}
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
