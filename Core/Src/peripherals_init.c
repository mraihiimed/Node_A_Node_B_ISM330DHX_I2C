/*
 * peripherals_init.c
 *
 *  Created on: Oct 9, 2025
 *      Author: Geek
 */

#include "peripherals_init.h"

void Init_All_Peripherals(void) {
    MX_GPIO_Init();
    MX_CAN_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();
    MX_TIM1_Init();
    SystemClock_Config();
}
