/*
 * can.h
 *
 *  Created on: Oct 9, 2025
 *      Author: Geek
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_
#include "main.h"
#include "motion_types.h"
void MX_CAN_Init(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void sendMotionStatusToNodeA(const MotionRaw_T* motion);
#endif /* INC_CAN_H_ */
