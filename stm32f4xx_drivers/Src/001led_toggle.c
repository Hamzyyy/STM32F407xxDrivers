/*
 * 001led_toggle.c
 *
 *  Created on: Apr 1, 2020
 *      Author: Hamzy
 */
#include <stdint.h>
#include "stm32f407xx.h"

// implement a delay ici
void delay(void)
{
	for(uint32_t i = 0; i < (500000/2); i++);
}

int main(void)
{
	// 1. Create a variable (object from GPIOHandle_t structure)
	GPIO_Handle_t gpio_led;

	// 2. Select the  GPIO port
	gpio_led.pGPIOx = GPIOD;

	// 3. Pin configuration through your object from structure
	gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;

	//gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

	gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD; //open drain

	//gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;  // as already push pull output type

	gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; // pull up

	gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;


	// 4. Enable the GPIO clock by passing the port & the clock value
	GPIO_PClkControl(GPIOD, ENABLE);

	// 5. Initialize the GPIO by passing the variable address
	GPIO_Init(&gpio_led);

	// 6. the super loop inside it the functionality
	while (1)
	{
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
		delay();
	}

	return 0;
}

