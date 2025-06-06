#include "timer.h"

/*
Register  | Description
TIMx_CR1  | Control register 1
TIMx_DIER | DMA/Interrupt enable register
TIMx_SR   | Status register
TIMx_PSC  | Pre-scaler
TIMx_ARR  | Auto-reload register (period)
TIMx_CNT  | Counter value
TIMx_EGR  | Event generation register
*/

volatile uint32_t delay_counter_t2 = 0;
volatile uint8_t delay_active_t2 = 0;

void TIM2_Init_1msTick(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;    	//Enable timer peripheral

    TIM2->PSC = 16000 - 1;  				// 16 MHz / 16000 = 1 kHz (1 ms tick)
    TIM2->ARR = 1;      				// Auto reload every 1 ms
    TIM2->CNT = 0;

    TIM2->DIER |= TIM_DIER_UIE;  			// Enable update interrupt
    TIM2->CR1 |= TIM_CR1_CEN;    			// Start timer

    NVIC_EnableIRQ(TIM2_IRQn);
}

//for non-blocking delay
/*
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF)
    {
        TIM2->SR &= ~TIM_SR_UIF;  // Clear interrupt flag
        if (delay_active_t2 && delay_counter_t2 > 0)
        {
            delay_counter_t2--;
            if (delay_counter_t2 == 0)
            {
                delay_active_t2 = 0;  // Mark delay complete
            }
        }
    }
}
*/

//for blocking delay
void TIM2_IRQHandler(void)
{
	if (TIM2->SR & TIM_SR_UIF)
	{
		TIM2->SR &= ~TIM_SR_UIF;  // Clear update flag
		if (delay_counter_t2 > 0)
		{
			delay_counter_t2--;
		}
	}
}

void delay_ms_nonblocking_t2(uint32_t ms)
{
    delay_counter_t2 = ms;
    delay_active_t2 = 1;
}

void delay_ms_blocking_t2(uint32_t ms)
{
	delay_counter_t2 = ms;
	while(delay_counter_t2 > 0);
}

uint8_t delay_expired_t2(void)
{
    return (delay_active_t2 == 0);
}

uint8_t delay_ms_t2(uint32_t ms)
{
	delay_ms_nonblocking_t2(ms);
	if(delay_expired_t2() == 1)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}


