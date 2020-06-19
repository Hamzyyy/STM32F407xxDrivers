/*
 * 003led_button.c
 *
 *  Created on: Apr 2, 2020
 *      Author: Hamzy
 */
/*
 * 002led_button.c
 *
 *  Created on: Apr 2, 2020
 *      Author: Hamzy
 */
#include <stdint.h>
#include <string.h>
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

	memset(&gpio_led, 0,sizeof(gpio_led)); // set every member element of the structure to 0
	memset(&gpio_button, 0,sizeof(gpio_button)); // set every member element of the structure to 0

	// 2. Select the  GPIO port for both  LED & Button
	gpio_led.pGPIOx = GPIOD;
	gpio_button.pGPIOx = GPIOD;

	// 3. Pin configuration through your object from structure (LED PIN)
	gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;        //LED on PA8
	gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;        //LED is output mode
	gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;    // output type is push pull better than open drain
    gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;  // as already push pull output type
	gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;     // does not really matter


	// 4. Pin configuration through your object from structure (Button PIN)
	gpio_button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;         //Button on PA0
	gpio_button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INT_FT;     //Button is interrupt falling edge trigger mode
	gpio_button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;   // as already pull down in the schematic
	gpio_button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;     // does not really matter


	// 5. Enable the GPIO clock by passing the port & the clock value (for both ports)
	GPIO_PClkControl(GPIOD, ENABLE);
	GPIO_PClkControl(GPIOD, ENABLE);

	// 6. Initialize the GPIO by passing the variable (address both ports)
	GPIO_Init(&gpio_led);
	GPIO_Init(&gpio_button);


	// 7. the IRQ configurations
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);

	// 8. the super loop inside it the functionality
	while (1)
	{

	}

	return 0;
}
void EXTI9_5_IRQHandler(void)
{
	delay(); //~200 ms
	GPIO_IRQHandling(GPIO_PIN_5); // clear the pending PR register
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12); // the action to do when the interrupt happen
}
