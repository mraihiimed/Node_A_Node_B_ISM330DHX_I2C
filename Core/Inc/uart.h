/*
 * uart.h
 *
 *  Created on: Oct 9, 2025
 *      Author: Geek
 */

#ifndef INC_UART_H_
#define INC_UART_H_
#include "main.h"

void uart_send_char(char c);
void uart_send_string(const char *str);
char uart_receive_char(void);
void MX_USART2_UART_Init(void);
int __io_putchar(int ch);

#endif /* INC_UART_H_ */
