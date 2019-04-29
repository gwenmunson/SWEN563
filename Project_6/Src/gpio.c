#include "gpio.h"

/*
 * GPIO_Init(void) - initializes GPIO pins for the timer
 */
void GPIO_Init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; //enable the peripheral clock of GPIO Port
    // Init PA1
    GPIOA->MODER &= ~(0x3 << 2);
    GPIOA->MODER |= (0x2 << 2);
    GPIOA->AFR[0] |= (0x2 << 4);

	//Init PA2
	GPIOA->MODER &= ~(0x03 << 4);
	GPIOA->MODER |= (0x02 << 4);
	GPIOA->AFR[0] |= (0x02 << 8);
}

