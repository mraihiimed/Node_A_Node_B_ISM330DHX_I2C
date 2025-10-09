/*
 * peripherals_init.h
 *
 *  Created on: Oct 9, 2025
 *      Author: Geek
 */

#ifndef INC_PERIPHERALS_INIT_H_
#define INC_PERIPHERALS_INIT_H_
//#include "main.h"
//#include "can.h"
//#include "tim.h"
//#include "gpio.h"
//#include "systemclock.h"
//#include "i2c1.h"
//#include "uart.h"




#include "stm32f1xx_hal.h"

// Peripheral handles (extern declarations)
extern CAN_HandleTypeDef hcan;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart2;

// Init functions
void MX_GPIO_Init(void);
void MX_CAN_Init(void);
void MX_I2C1_Init(void);
void MX_USART2_UART_Init(void);
void MX_TIM1_Init(void);
void SystemClock_Config(void);

// Unified init wrapper
void Init_All_Peripherals(void);



#endif /* INC_PERIPHERALS_INIT_H_ */
