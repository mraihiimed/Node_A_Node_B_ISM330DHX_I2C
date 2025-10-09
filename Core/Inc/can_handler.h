/*
 * can_handler.h
 *
 *  Created on: Oct 9, 2025
 *      Author: Geek
 */

#ifndef INC_CAN_HANDLER_H_
#define INC_CAN_HANDLER_H_
#include "main.h"
#include "motion_types.h"
void sendMotionStatusToNodeA(const MotionRaw_T* motion);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
#endif /* INC_CAN_HANDLER_H_ */
