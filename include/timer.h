

#ifndef TIMER_H_
#define TIMER_H_

#include "stm32f407xx.h"

void TIM2_Init_1msTick(void);
void TIM2_IRQHandler(void);

void delay_ms_nonblocking_t2(uint32_t ms);
uint8_t delay_expired_t2(void);
uint8_t delay_ms_t2(uint32_t ms);

void delay_ms_blocking_t2(uint32_t ms);

#endif // TIMER_H_
