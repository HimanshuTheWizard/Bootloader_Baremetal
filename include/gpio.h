#ifndef GPIO_H_
#define GPIO_H_

#include "stm32f407xx.h"  // Include device-specific header

#define GPIO_PIN_SET 		1
#define GPIO_PIN_RESET 		0

typedef enum {
    GPIO_MODE_INPUT  = 0x00,
    GPIO_MODE_OUTPUT = 0x01,
    GPIO_MODE_AF     = 0x02,
    GPIO_MODE_ANALOG = 0x03
} GPIO_Mode;

typedef enum {
    GPIO_OUTPUT_PP = 0x00,
    GPIO_OUTPUT_OD = 0x01
} GPIO_OutputType;

typedef enum {
    GPIO_SPEED_LOW    = 0x00,
    GPIO_SPEED_MEDIUM = 0x01,
    GPIO_SPEED_FAST   = 0x02,
    GPIO_SPEED_HIGH   = 0x03
} GPIO_Speed;

typedef enum {
    GPIO_NOPULL = 0x00,
    GPIO_PULLUP = 0x01,
    GPIO_PULLDOWN = 0x02
} GPIO_Pull;

typedef struct {
    GPIO_TypeDef *port;
    uint8_t pin;
    GPIO_Mode mode;
    GPIO_OutputType otype;
    GPIO_Speed speed;
    GPIO_Pull pull;
} GPIO_Config;



void GPIO_Init(GPIO_Config *cfg);
void GPIO_SetPin(GPIO_TypeDef *port, uint8_t pin);
void GPIO_ResetPin(GPIO_TypeDef *port, uint8_t pin);
void GPIO_TogglePin(GPIO_TypeDef *port, uint8_t pin);
uint8_t GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);

#endif

