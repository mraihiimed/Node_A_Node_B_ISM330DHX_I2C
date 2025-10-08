/*
 * dht.h
 *
 *  Created on: Oct 8, 2025
 *      Author: Geek
 */

#ifndef INC_DHT_H_
#define INC_DHT_H_


#include "stm32f1xx_hal_tim.h"


#define DHT11_PORT GPIOB
#define DHT11_PIN GPIO_PIN_9

extern TIM_HandleTypeDef htim1;
extern uint8_t RHI, RHD, TCI, TCD, SUM;
extern uint32_t pMillis, cMillis;
extern float tCelsius;
extern float tFahrenheit;
extern float RH;

uint8_t DHT11_Read (void);
uint8_t DHT11_Start (void);
void microDelay(uint16_t delay);

#endif /* INC_DHT_H_ */
