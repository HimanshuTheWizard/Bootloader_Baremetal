/*
 * uart.h
 *
 *  Created on: 15-Apr-2025
 *      Author: himanshu
 */

#ifndef UART_H_
#define UART_H_

#include "stm32f407xx.h"

typedef enum {
    USART_MODE_TX = 0x08,
    USART_MODE_RX = 0x04,
    USART_MODE_TX_RX = 0x0C
} USART_Mode;

typedef struct {
	USART_TypeDef *Instance;  // USART2 or USART3
    uint32_t BaudRate;
    USART_Mode Mode;
} USART_Config;

void USART_Init(USART_Config *cfg);
void USART_SendChar(USART_TypeDef *usart, char c);
void USART_SendString(USART_TypeDef *usart, const char *str);
char USART_ReceiveChar(USART_TypeDef *usart);
void USART_ReceiveString(USART_TypeDef *usart, char *str);

#endif /* UART_H_ */
