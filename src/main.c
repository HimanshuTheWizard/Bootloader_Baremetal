#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "diag/trace.h"
#include "uart.h"
#include "gpio.h"
#include "timer.h"

/*==========================local macros==========================*/
#define COMMAND_BL_GET_VER								0x51 //Completed
#define COMMAND_BL_GET_HELP								0x52
#define COMMAND_BL_GET_CID								0x53
#define COMMAND_BL_GET_RDP_STATUS						0x54
#define COMMAND_BL_GO_TO_ADDR							0x55
#define COMMAND_BL_FLASH_ERASE							0x56 //Completed
#define COMMAND_BL_MEM_WRITE							0x57 //Completed
#define COMMAND_BL_EN_R_W_PROTECT						0x58
#define COMMAND_BL_MEM_READ								0x59
#define COMMAND_BL_READ_SECTOR_P_STATUS					0x5A
#define COMMAND_BL_OTP_READ								0x5B
#define COMMAND_BL_DIS_R_W_PROTECT						0x5C
#define COMMAND_BL_MY_NEW_COMMAND                      	0x5D

/*len details of the command*/
#define COMMAND_BL_GET_VER_LEN                              2
#define COMMAND_BL_GET_HELP_LEN                             6
#define COMMAND_BL_GET_CID_LEN                              6
#define COMMAND_BL_GET_RDP_STATUS_LEN                       6
#define COMMAND_BL_GO_TO_ADDR_LEN                           10
#define COMMAND_BL_FLASH_ERASE_LEN                          4
#define COMMAND_BL_MEM_WRITE_LEN                            7
#define COMMAND_BL_EN_R_W_PROTECT_LEN                       8
#define COMMAND_BL_READ_SECTOR_P_STATUS_LEN                 6
#define COMMAND_BL_DIS_R_W_PROTECT_LEN                      6
#define COMMAND_BL_MY_NEW_COMMAND_LEN                       8

#define PROCESSOR_CLOCK			    					(16000000)				//16MHz
#define PROCESSOR_CLOCK_MS		    					(PROCESSOR_CLOCK/1000)

#define FLASH_SECTOR2_BASE_ADDRESS  					0x08008000
#define FLASH_SECTOR6_BASE_ADDRESS						0x08040000

#define VERSION_MAJOR   1
#define VERSION_MINOR   0

#define METADATA_SECTOR 11
#define BOOT_METADATA_ADDR ((uint32_t)0x080E0000)  // Choose a safe sector (11)

typedef struct __attribute__((__packed__)) {
    uint8_t boot_flag;      // Use uint8_t for 0xB007B007
    uint8_t active_app;      // 1 or 2
    uint8_t reserved[3];     // Padding to make structure 8 bytes (aligned to word)
} Boot_Metadata;

/*==========================Global Variables==========================*/
uint8_t pBuffer[200];
volatile uint8_t indicator_flag = 0;

/*==========================Function Prototype==========================*/
void Jump_To_Bootloader_Code(Boot_Metadata boot_data);
void Jump_To_Application_Code(Boot_Metadata boot_data);
void BL_Flash_Erase(uint8_t *buffer);
void BL_Flash_Write(uint8_t *buffer, Boot_Metadata boot_data);
void BL_Flash_Read(uint8_t *buffer);
void BL_Get_Ver(uint8_t *pBuffer, uint8_t major, uint8_t minor);
uint8_t CRC_Calculation_And_Verification(uint8_t *pBuffer, int len, uint32_t expected_crc);
int Char2Int(char c);
Boot_Metadata Read_Boot_Metadata(void);
void WriteBootMetadata(uint8_t boot_flag, uint8_t active_app);


int main()
{
	Boot_Metadata boot_data = {0};

	boot_data = Read_Boot_Metadata();

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
		if((GPIO_ReadPin(GPIOA, 0) == GPIO_PIN_SET) || (boot_data.boot_flag == 1))
		{
			Jump_To_Bootloader_Code(boot_data);
		}
		else
		{
			Jump_To_Application_Code(boot_data);
		}
	}
}
/*==========================Function Definitions==========================*/
void Jump_To_Bootloader_Code(Boot_Metadata boot_data)
{
	uint8_t dummy;
	dummy = USART_ReceiveChar(USART2);
	//send to user which version of FW is active and running
	USART_SendChar(USART2, boot_data.active_app);
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
				BL_Get_Ver(pBuffer, VERSION_MAJOR, VERSION_MINOR);
				break;
			case COMMAND_BL_MEM_WRITE:
				//BL_MEM_WRITE
				BL_Flash_Write(pBuffer, boot_data);
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

void BL_Get_Ver(uint8_t *pBuffer, uint8_t major, uint8_t minor)
{
	int ack = 0xA5; //When CRC is implemented, hard-coded ack can be updated
	int len_to_follow = 2;
	USART_SendChar(USART2, ack); //major and minor
	USART_SendChar(USART2, len_to_follow);
	USART_SendChar(USART2, major);
	USART_SendChar(USART2, minor);
}

void BL_Flash_Write(uint8_t *buffer, Boot_Metadata boot_data)
{
    const uint32_t KEY1 = 0x45670123;
    const uint32_t KEY2 = 0xCDEF89AB;
    uint32_t target_address = ((buffer[2]) | (buffer[3] << 8) | (buffer[4] << 16) | (buffer[5] << 24));

    int i = COMMAND_BL_MEM_WRITE_LEN;
    int len_to_follow = buffer[0];
    int payload_len = len_to_follow - COMMAND_BL_MEM_WRITE_LEN;

    int padded_len = (payload_len + 3) & ~0x03; // Round to 4-byte boundary

    // Unlock flash
    if (FLASH->CR & FLASH_CR_LOCK)
    {
        FLASH->KEYR = KEY1;
        FLASH->KEYR = KEY2;
    }

    // Set PSIZE to 32-bit
    FLASH->CR &= ~(FLASH_CR_PSIZE);
    FLASH->CR |= (0x2 << 8); // PSIZE = 32-bit
    FLASH->CR |= FLASH_CR_PG;

    while (padded_len > 0)
    {
        uint32_t word = 0xFFFFFFFF;

        for (int b = 0; b < 4; b++)
		{
			int src_index = i + b;
			// Reverse bytes: store buffer[i] into MSB, buffer[i+3] into LSB
			((uint8_t*)&word)[b] = buffer[src_index];
		}

        *(volatile uint32_t*)target_address = word;

        while (FLASH->SR & FLASH_SR_BSY);

        if (FLASH->SR & FLASH_SR_EOP)
            FLASH->SR = FLASH_SR_EOP;

        target_address += 4;
        i += 4;
        padded_len -= 4;
    }

    FLASH->CR &= ~FLASH_CR_PG;
    FLASH->CR |= FLASH_CR_LOCK;

    USART_SendChar(USART2, 0xA5);
    USART_SendChar(USART2, 1);
    USART_SendChar(USART2, 0);


    if((boot_data.active_app == 1) && (boot_data.boot_flag == 1))
    {
    	WriteBootMetadata(0, 2);
    }
    else if((boot_data.active_app == 2) && (boot_data.boot_flag == 1))
    {
    	WriteBootMetadata(0, 1);
    }
	else
	{
		//do  nothing
	}
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
	USART_SendChar(USART2, 0xA5);
	USART_SendChar(USART2, 1);
	USART_SendChar(USART2, 0);
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
#define APP1_ADD   (uint32_t)0x08008000
#define APP2_ADD   (uint32_t)0x08040000
void Jump_To_Application_Code(Boot_Metadata boot_data_l)
{
	uint32_t APPLICATION_BASE_ADDRESS;

	//if this is first time system is booting to application after flash
	if(boot_data_l.active_app == 0xFF)
	{
		if (*(uint32_t *)APP1_ADD == 0xFFFFFFFF)
		{
		    APPLICATION_BASE_ADDRESS = FLASH_SECTOR6_BASE_ADDRESS;
		}
		else if (*(uint32_t *)APP2_ADD == 0xFFFFFFFF)
		{
		    APPLICATION_BASE_ADDRESS = FLASH_SECTOR2_BASE_ADDRESS;
		}
		else
		{
		    // do nothing
		}
	}
	else
	{
		if (boot_data_l.active_app == 1)
			APPLICATION_BASE_ADDRESS = FLASH_SECTOR2_BASE_ADDRESS;
		else
			APPLICATION_BASE_ADDRESS = FLASH_SECTOR6_BASE_ADDRESS;
	}

	//1. init stack pointer
	uint32_t sp = *(volatile uint32_t*)(APPLICATION_BASE_ADDRESS);
	__set_MSP(sp);

	SCB->VTOR = APPLICATION_BASE_ADDRESS;

	//2. init and called reset handler
	void (*Reset_Handler)(void);
	uint32_t reset_handler_t;
	reset_handler_t = *(volatile uint32_t*)(APPLICATION_BASE_ADDRESS + 4);
	Reset_Handler = (void *)reset_handler_t;
	Reset_Handler();
}

// Expected: CRC32 with 0x04C11DB7 poly, initial value 0xFFFFFFFF, bit-reversed IO (hardware default)
//this function is not is use for now, as CRC is not implemented
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

Boot_Metadata Read_Boot_Metadata(void)
{
    return *(Boot_Metadata *)BOOT_METADATA_ADDR;
}

void WriteBootMetadata(uint8_t boot_flag, uint8_t active_app)
{
    uint32_t FLASH_METADATA_ADDR = BOOT_METADATA_ADDR;
    const uint32_t KEY1 = 0x45670123;
    const uint32_t KEY2 = 0xCDEF89AB;

    // Unlock flash
    if (FLASH->CR & FLASH_CR_LOCK)
    {
        FLASH->KEYR = KEY1;
        FLASH->KEYR = KEY2;
    }

    // Erase sector
    FLASH->CR &= ~FLASH_CR_SNB;
    FLASH->CR |= (METADATA_SECTOR << 3); // sector 11
    FLASH->CR |= FLASH_CR_SER;
    FLASH->CR |= FLASH_CR_STRT;
    while (FLASH->SR & FLASH_SR_BSY);
    FLASH->CR &= ~FLASH_CR_SER;

    // Set programming size to 32-bit
    FLASH->CR &= ~(FLASH_CR_PSIZE);
    FLASH->CR |= (2U << 8); // PSIZE = 32-bit

    // Write data
    FLASH->CR |= FLASH_CR_PG;

    Boot_Metadata metadata = { boot_flag, active_app, {0} };
    uint32_t *src = (uint32_t *)&metadata;
    volatile uint32_t *dst = (uint32_t *)FLASH_METADATA_ADDR;

    dst[0] = src[0];
    dst[1] = src[1];

    while (FLASH->SR & FLASH_SR_BSY);

    FLASH->CR &= ~FLASH_CR_PG;
    FLASH->CR |= FLASH_CR_LOCK;
}


