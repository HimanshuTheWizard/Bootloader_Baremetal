#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "diag/trace.h"
#include "uart.h"
#include "gpio.h"
#include "timer.h"

/*local macros*/
#define COMMAND_BL_GET_VER								0x51 //Completed
#define COMMAND_BL_GET_HELP								0x52
#define COMMAND_BL_GET_CID								0x53
#define COMMAND_BL_GET_RDP_STATUS						0x54
#define COMMAND_BL_GO_TO_ADDR							0x55
#define COMMAND_BL_FLASH_ERASE							0x56
#define COMMAND_BL_MEM_WRITE							0x57
#define COMMAND_BL_EN_R_W_PROTECT						0x58
#define COMMAND_BL_MEM_READ								0x59
#define COMMAND_BL_READ_SECTOR_P_STATUS					0x5A
#define COMMAND_BL_OTP_READ								0x5B
#define COMMAND_BL_DIS_R_W_PROTECT						0x5C
#define COMMAND_BL_MY_NEW_COMMAND                      	0x5D
#define PROCESSOR_CLOCK			    					(16000000)				//16MHz
#define PROCESSOR_CLOCK_MS		    					(PROCESSOR_CLOCK/1000)
#define FLASH_SECTOR2_BASE_ADDRESS  					0x08008000

#define VERSION_MAJOR   1
#define VERSION_MINOR   0

/*Global Variables*/
uint8_t pBuffer[200];

/*Function Prototype*/
void Jump_To_Bootloader_Code(void);
void Jump_To_Application_Code(void);
void BL_Flash_Erase(uint8_t *buffer);
void BL_Flash_Write(uint8_t *buffer);
void BL_Flash_Read(uint8_t *buffer);
void BL_Get_Ver(uint8_t major, uint8_t minor);
uint8_t CRC_Calculation_And_Verification(uint8_t *pBuffer, int len, uint32_t expected_crc);
int Char2Int(char c);

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
		//1. Receive length byte
		pBuffer[0] = USART_ReceiveChar(USART2);
		uint8_t len = pBuffer[0];
		int i = 1;

		//2. Read rest of the packet
		while(len--)
		{
		    pBuffer[i++] = USART_ReceiveChar(USART2);
		}


		//3. Extract CMD Code and length
		uint8_t CMD_Code = pBuffer[1];

		switch(CMD_Code)
		{
			case COMMAND_BL_GET_VER:
				//BL_Gen_VER
				BL_Get_Ver(VERSION_MAJOR, VERSION_MINOR);
				break;
			case COMMAND_BL_MEM_WRITE:
				//BL_MEM_WRITE
				BL_Flash_Write(pBuffer);
				break;
			case COMMAND_BL_FLASH_ERASE:
				//BL_FLASH_ERASE
				BL_Flash_Erase(pBuffer);
				break;
			default:
				USART_SendString(USART2, "Incorrect CMD Code\n");
				break;
		}
	}
}

void BL_Get_Ver(uint8_t major, uint8_t minor)
{
	int ack = 0xA5; //When CRC is implemented, hard-coded ack can be updated
	int len_to_follow = 2;
	USART_SendChar(USART2, ack); //major and minor
	USART_SendChar(USART2, len_to_follow);
	USART_SendChar(USART2, major);
	USART_SendChar(USART2, minor);
}
void BL_Flash_Write(uint8_t *buffer)
{

}
void BL_Flash_Erase(uint8_t *buffer)
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

// Expected: CRC32 with 0x04C11DB7 poly, initial value 0xFFFFFFFF, bit-reversed IO (hardware default)
uint8_t CRC_Calculation_And_Verification(uint8_t *pBuffer, int len, uint32_t expected_crc)
{
	uint8_t ret_val = 0;

    // 1. Enable CRC peripheral clock
    RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;

    // 2. Reset CRC computation unit
    CRC->CR |= CRC_CR_RESET;

    // 3. Feed data as 32-bit words (LSB first due to internal input bit reversal)
    for (int i = 0; i < len;)
    {
        uint32_t data = 0;

        for (int j = 0; j < 4 && i < len; j++, i++)
        {
            data |= ((uint8_t)pBuffer[i] << (8 * j));
        }

        CRC->DR = data;
    }

    // 4. Read the CRC result
    uint32_t computed_crc = CRC->DR;

    // 5. Disable CRC peripheral (optional)
    RCC->AHB1ENR &= ~RCC_AHB1ENR_CRCEN;

    // 6. Print result (trace or UART)
    if (computed_crc == expected_crc)
    {
    	ret_val = 1;
    }
    return ret_val;
}

int Char2Int(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    else
        return -1; // Invalid input
}
