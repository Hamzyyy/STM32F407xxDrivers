/*
 * 010i2c_master_rx.c
 *
 *  Created on: Oct 23, 2020
 *      Author: Hamzy
 */

#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"


// we have three messages that will be transmitted to the arduino

// array of strings
char *msg[3] = {"hamzy is so cool!", " How are you today?", "Aujourd'hui c'est le venderdi"};

// reply from arduino will be stored here
char rx_buffer[1024];


// flag indicates the reception completion
uint8_t rxCmplt = RESET;

uint8_t g_data = RESET;

//semi-hosting
extern void initialise_monitor_handles();

USART_Handle_t usart2_handle;



void USART2_GPIOInits(void)
{
	GPIO_Handle_t usart_pins;

	usart_pins.pGPIOx = GPIOA;
	usart_pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart_pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart_pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	usart_pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart_pins.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF7;

	//USART2 ---> TX
	usart_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_2;
	//Init this pin
	GPIO_Init(&usart_pins);

	//USART2 ---> RX
	usart_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_3;
	//Init this pin
	GPIO_Init(&usart_pins);
}


void GPIO_ButtonInits(void)
{
	GPIO_Handle_t gpio_button, gpio_led;

	gpio_button.pGPIOx = GPIOA;
	// Pin configuration through your object from structure (Button PIN)
	gpio_button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;         //Button on PA0
	gpio_button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;         //Button is input mode
	gpio_button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;  // as already pull down in the schematic
	gpio_button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;     // does not really matter


	GPIO_Init(&gpio_button);



	// 3. Pin configuration through your object from structure (LED PIN)
	gpio_led.pGPIOx = GPIOD;

	gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;        //LED on PD12
	gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;        //LED is output mode
	gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;    // output type is push pull better than open drain
    gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;  // as already push pull output type
	gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;     // does not really matter

	GPIO_Init(&gpio_led);
}


void USART2_Inits(void)
{
	// Choose the peripheral
	usart2_handle.pUSARTx = USART2;

	// Configure USART2 peripheral
	usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2_handle.USART_Config.USART_Mode = USART_MODE_TX_RX;
	usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;

	// Init the USART
	USART_Init( &usart2_handle);
}

void delay(void)
{
	for(uint32_t i = 0; i < (500000/2); i++);
}



int main(void)
{

	uint32_t count = RESET;


	initialise_monitor_handles();

	// Button Configuration
	GPIO_ButtonInits();

	// USART pins configurations
	USART2_GPIOInits();


	// USART peripheral Initialization
	USART2_Inits();

	// configure the USART2 interrupt
	USART_IRQInterruptConfig(IRQ_NO_USART2, ENABLE);

	USART_PeripheralControl(USART2, ENABLE);


    printf("Application is running\n");


	while(1)
	{
		// wait until the button is pressed
		while ( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0) );

		delay();

		// Next message index ; make sure that cnt value doesn't cross 2
		count = count % 3;


		//First lets enable the reception in interrupt mode
		//this code enables the receive interrupt
		while ( USART_ReceiveDataIT( &usart2_handle, rx_buffer, strlen(msg[count]) ) != USART_READY );

		//Send the msg indexed by cnt in blocking mode
		USART_SendData(&usart2_handle, (uint8_t*) msg[count], strlen(msg[count]));

    	printf("Transmitted : %s\n",msg[count]);


    	//Now lets wait until all the bytes are received from the arduino.
    	//When all the bytes are received rxCmplt will be SET in application callback
    	while(rxCmplt != SET);


    	//Print what we received from the arduino
    	printf("Received    : %s\n",rx_buffer);

    	//invalidate the flag
    	rxCmplt = RESET;

    	//move on to next message indexed in msg[]
    	count++;
	}


	return 0;
}

void USART2_IRQHandler(void)
{
	USART_IRQHandling( &usart2_handle);
}

void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t ApEv)
{
	if(ApEv == USART_EVENT_RX_CMPLT)
	{
		rxCmplt = SET;
	}
	else if(ApEv == USART_EVENT_TX_CMPLT)
	{

	}
}
