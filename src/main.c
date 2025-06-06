#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "diag/trace.h"
#include "uart.h"
#include "gpio.h"
#include "timer.h"

/*local macros*/
#define PROCESSOR_CLOCK			    (16000000)				//16MHz
#define PROCESSOR_CLOCK_MS		    (PROCESSOR_CLOCK/1000)
#define FLASH_SECTOR2_BASE_ADDRESS  0x08008000

#define VERSION_MAJOR   '1'
#define VERSION_MINOR   '0'

/*Global Variables*/
char pBuffer[200];

/*Function Prototype*/
void Jump_To_Bootloader_Code(void);
void Jump_To_Application_Code(void);
void BL_Flash_Erase(char *buffer);
void BL_Flash_Write(char *buffer);
void BL_Flash_Read(char *buffer);
void BL_Get_Ver(char *buffer, uint8_t major, uint8_t minor);

int main()
{
	//1. Initialize user button on board - PA0
	GPIO_Config User_Button_Config =
	{
		.port = GPIOA,
		.pin = 0,
		.mode = GPIO_MODE_INPUT,
		.speed = GPIO_SPEED_MEDIUM,
		.pull = GPIO_PULLDOWN,
	};

	GPIO_Init(&User_Button_Config);

	//2. Initialize UART2 and UART3
	USART_Config UART2_Config = {USART2, 9600, USART_MODE_TX_RX};
	USART_Config UART3_Config = {USART3, 9600, USART_MODE_TX_RX};

	GPIO_Config tx2 = {GPIOA, 2, GPIO_MODE_AF, GPIO_OUTPUT_PP, GPIO_SPEED_HIGH, GPIO_NOPULL};
	GPIO_Config rx2 = {GPIOA, 3, GPIO_MODE_AF, GPIO_OUTPUT_PP, GPIO_SPEED_HIGH, GPIO_NOPULL};

	GPIO_Config tx3 = {GPIOB, 10, GPIO_MODE_AF, GPIO_OUTPUT_PP, GPIO_SPEED_HIGH, GPIO_NOPULL};
	GPIO_Config rx3 = {GPIOB, 11, GPIO_MODE_AF, GPIO_OUTPUT_PP, GPIO_SPEED_HIGH, GPIO_NOPULL};

	GPIO_Init(&tx2);
	GPIO_Init(&rx2);

	GPIO_Init(&tx3);
	GPIO_Init(&rx3);


	//3. Initialize UART specific GPIO alternate function
	GPIOA->AFR[0] &= ~((0xF << (4 * 2)) | (0xF << (4 * 3)));
	GPIOA->AFR[0] |=  (7 << (4 * 2)) | (7 << (4 * 3));

	GPIOB->AFR[1] &= ~((0xF << (4 * (10 - 8))) | (0xF << (4 * (11 - 8)))); // Clear bits
	GPIOB->AFR[1] |=  (7 << (4 * (10 - 8))) | (7 << (4 * (11 - 8)));       // Set AF7

	USART_Init(&UART2_Config);
	USART_Init(&UART3_Config);

	while (1)
	{
		if(GPIO_ReadPin(GPIOA, 0) == GPIO_PIN_SET)
		{
			Jump_To_Bootloader_Code();
		}
		else
		{
			Jump_To_Application_Code();
		}
	}
}


void Jump_To_Bootloader_Code(void)
{
	while(1)
	{
		// 1. Receive a string terminated by '\n'
		USART_ReceiveString(USART2, pBuffer);

		// to receive back echo on terminal
		USART_SendString(USART2, pBuffer);

		USART_SendString(USART2, "\r\n");

		//3. Extract CMD Code
		char CMD_Code = pBuffer[1];

		switch(CMD_Code)
		{
			case 'A':
				//BL_Gen_VER
				BL_Get_Ver(pBuffer, VERSION_MAJOR, VERSION_MINOR);
				break;
			case 'C':
				//BL_MEM_WRITE
				BL_Flash_Write(pBuffer);
				break;
			case 'D':
				//BL_FLASH_ERASE
				BL_Flash_Erase(pBuffer);
				break;
			default:
				USART_SendString(USART2, "Incorrect CMD Code\n");
				break;
		}
	}
}
void BL_Get_Ver(char *buffer, uint8_t major, uint8_t minor)
{
	//Additionally you can add CRC
	USART_SendString(USART2, "Version :");
	USART_SendChar(USART2, major);
	USART_SendChar(USART2, minor);
	USART_SendString(USART2, "\r\n");
}
void BL_Flash_Write(char *buffer)
{

}
void BL_Flash_Erase(char *buffer)
{
	uint32_t KEY1 = 0x45670123;
	uint32_t KEY2 = 0xCDEF89AB;
	uint32_t sector_num = pBuffer[2];
	uint32_t num_of_sec = pBuffer[3];

	//1. Unlock the flash
	FLASH->KEYR = KEY1;
	FLASH->KEYR = KEY2;
	//any wrong sequence returns the bus error, check the same.

	while(num_of_sec)
	{
		//2. Check that no flash memory operation is ongoing
		if((FLASH->SR & (1 << 16)) == 0U)
		{
			//3. set SER bit in flash_CR register
			FLASH->CR |= (1<<1);

			//4. select the sector to erase
			FLASH->CR |= (sector_num << 3);

			//5. Set the start bit in CR
			FLASH->CR |= (1<<16);

			//6. wait for busy bit to get cleared
			while((FLASH->SR & (1<<16)) != 0U);
		}
		num_of_sec--;
		sector_num++;
	}
	//7. lock the flash
	FLASH->CR |= (1<<31);
}

void Jump_To_Application_Code(void)
{
	//1. init stack pointer
	uint32_t sp = *(volatile uint32_t*)(FLASH_SECTOR2_BASE_ADDRESS);
	__set_MSP(sp);

	//2. init and called reset handler
	void (*Reset_Handler)(void);
	uint32_t reset_handler_t;
	reset_handler_t = *(volatile uint32_t*)(FLASH_SECTOR2_BASE_ADDRESS + 4);
	Reset_Handler = (void *)reset_handler_t;
	Reset_Handler();
}



/*
 * Testing
 * 1. GPIO set/reset/toggle
 * 2. UART2 read write (blocking)
 * 3. UART3 read write (blocking)
 */

//1. LD6 Blue - PD15
/*
GPIO_Config Blue_Led_Config = {
	.port = GPIOD,
	.pin = 15,
	.mode = GPIO_MODE_OUTPUT,
	.otype = GPIO_OUTPUT_PP,
	.speed = GPIO_SPEED_MEDIUM,
	.pull = GPIO_PULLDOWN,
};

GPIO_Init(&Blue_Led_Config);
TIM2_Init_1msTick();
while(1)
{
	GPIO_SetPin(GPIOD, 15);
	delay_ms_blocking_t2(1000);
	GPIO_ResetPin(GPIOD, 15);
	delay_ms_blocking_t2(1000);
}
=============================================================
// 1. USART2 Configuration
USART_Config UART2_Config = {USART2, 9600, USART_MODE_TX_RX};

// 2. GPIO Configuration for USART2 (TX = PA2, RX = PA3)
GPIO_Config tx = {GPIOA, 2, GPIO_MODE_AF, GPIO_OUTPUT_PP, GPIO_SPEED_HIGH, GPIO_NOPULL};
GPIO_Config rx = {GPIOA, 3, GPIO_MODE_AF, GPIO_OUTPUT_PP, GPIO_SPEED_HIGH, GPIO_NOPULL};

// 3. Initialize GPIOs
GPIO_Init(&tx);
GPIO_Init(&rx);

// 4. Set alternate function AF7 for USART2 on PA2/PA3
GPIOA->AFR[0] &= ~((0xF << (4 * 2)) | (0xF << (4 * 3)));
GPIOA->AFR[0] |=  (7 << (4 * 2)) | (7 << (4 * 3));

// 5. Initialize USART2
USART_Init(&UART2_Config);

*/











