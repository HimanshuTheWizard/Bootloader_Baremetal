#include "gpio.h"

void GPIO_Init(GPIO_Config *cfg) {
    // 1. Enable peripheral clock
    if (cfg->port == GPIOA) 	 RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    else if (cfg->port == GPIOB) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    else if (cfg->port == GPIOC) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    else if (cfg->port == GPIOD) RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    else if (cfg->port == GPIOE) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    else if (cfg->port == GPIOF) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
    else if (cfg->port == GPIOG) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
    else if (cfg->port == GPIOH) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
    else if (cfg->port == GPIOI) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN;

    uint32_t pin_pos = cfg->pin * 2;

    // 2. Set MODER
    cfg->port->MODER &= ~(0x3U << pin_pos); 			//clear the bits
    cfg->port->MODER |= ((cfg->mode & 0x3U) << pin_pos);

    // 3. Set OTYPER (only valid for output/AF modes)
    if (cfg->mode == GPIO_MODE_OUTPUT || cfg->mode == GPIO_MODE_AF) {
        cfg->port->OTYPER &= ~(0x1U << cfg->pin); 		//clear the bits
        cfg->port->OTYPER |= ((cfg->otype & 0x1U) << cfg->pin);
    }

    // 4. Set OSPEEDR
    cfg->port->OSPEEDR &= ~(0x3U << pin_pos);			//clear the bits
    cfg->port->OSPEEDR |= ((cfg->speed & 0x3U) << pin_pos);

    // 5. Set PUPDR
    cfg->port->PUPDR &= ~(0x3U << pin_pos);				//clear the bits
    cfg->port->PUPDR |= ((cfg->pull & 0x3U) << pin_pos);
}

// Set GPIO pin HIGH
void GPIO_SetPin(GPIO_TypeDef *port, uint8_t pin) {
    port->BSRR = (1U << pin);  // Set bit
}

// Set GPIO pin LOW
void GPIO_ResetPin(GPIO_TypeDef *port, uint8_t pin) {
    port->BSRR = (1U << (pin + 16));  // Reset bit
}

// Toggle GPIO pin
void GPIO_TogglePin(GPIO_TypeDef *port, uint8_t pin) {
    port->ODR ^= (1U << pin);  // XOR to toggle
}

uint8_t GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin) {
    return (port->IDR & (1U << pin)) ? 1 : 0;
}


