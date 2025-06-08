#include "uart.h"

void USART_Init(USART_Config *cfg) {
    // 1. Enable USART peripheral clock
    if (cfg->Instance == USART2)
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    else if (cfg->Instance == USART3)
        RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

    // 2. Calculate baud rate (Assuming APB1 = 16 MHz default)
    uint32_t pclk1 = 16000000U;
    uint32_t usartdiv = (pclk1 + (cfg->BaudRate / 2U)) / cfg->BaudRate;  // rounding
    cfg->Instance->BRR = usartdiv;

    // 3. Configure Word length, Parity, Stop bits (if needed)
    cfg->Instance->CR1 &= ~(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS); // 8-bit, no parity

    // 4. Enable USART mode (TX and/or RX)
    cfg->Instance->CR1 &= ~(USART_CR1_TE | USART_CR1_RE);
    cfg->Instance->CR1 |= cfg->Mode;

    // 5. Enable USART
    cfg->Instance->CR1 |= USART_CR1_UE;
}


void USART_SendChar(USART_TypeDef *usart, char c) {
    while (!(usart->SR & USART_SR_TXE));  // Wait until TXE = 1
    usart->DR = (c & 0xFF);
}

void USART_SendString(USART_TypeDef *usart, const uint8_t *str) {
    while (*str) {
        USART_SendChar(usart, *str++);
    }
}

char USART_ReceiveChar(USART_TypeDef *usart) {
    while (!(usart->SR & USART_SR_RXNE));  // Wait until RXNE = 1
    return (char)(usart->DR & 0xFF);
}

void USART_ReceiveString(USART_TypeDef *usart, uint8_t *str) {
	char chr;
    do
    {
    	chr = USART_ReceiveChar(usart);
    	*str = chr;
    	str++;
    }
    while(chr != '\r' && chr != '\n' && chr != ' ');
    *(str - 1) = '\0';
}


