/*
 * 002led_button.c
 *
 *  Created on: Apr 2, 2020
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
	//LED variable & Button variable
	GPIO_Handle_t gpio_led, gpio_button;

	// 2. Select the  GPIO port for both  LED & Button
	gpio_led.pGPIOx = GPIOD;
	gpio_button.pGPIOx = GPIOA;

	// 3. Pin configuration through your object from structure (LED PIN)
	gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;        //LED on PD12
	gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;        //LED is output mode
	gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;    // output type is push pull better than open drain
    gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;  // as already push pull output type
	gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;     // does not really matter


	// 3. Pin configuration through your object from structure (Button PIN)
	gpio_button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;         //Button on PA0
	gpio_button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;         //Button is input mode
	gpio_button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;  // as already pull down in the schematic
	gpio_button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;     // does not really matter


	// 5. Enable the GPIO clock by passing the port & the clock value (for both ports)
	GPIO_PClkControl(GPIOD, ENABLE);
	GPIO_PClkControl(GPIOA, ENABLE);

	// 6. Initialize the GPIO by passing the variable (address both ports)
	GPIO_Init(&gpio_led);
	GPIO_Init(&gpio_button);

	// 6. the super loop inside it the functionality
	while (1)
	{
		if (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0)) // if the button is pressed GPIO_ReadFromInputPin ==1
		{
			delay();                                 //in order to debouncing
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
		}

		else
		{
			GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_12, RESET);
		}
	}

	return 0;
}



