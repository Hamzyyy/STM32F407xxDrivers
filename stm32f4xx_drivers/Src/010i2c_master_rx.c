/*
 * 010i2c_master_rx.c
 *
 *  Created on: Aug 29, 2020
 *      Author: Hamzy
 */

#include<stdio.h>
#include <string.h>
#include "stm32f407xx.h"


extern void initialise_monitor_handles();


/*  Pin Configurations
 * PB6 ---> I2C_SCL
 * PB7 ---> I2C_SDA
 * ALT Function = 4
 */

#define MY_ADDR   0x61
#define SLAVE_ADDR  0x68

I2C_Handle_t I2C1Handle;

uint8_t received_buffer[32];



void I2C1_GPIOInits(void)
{
	GPIO_Handle_t i2c_pins;

	i2c_pins.pGPIOx = GPIOB;
	i2c_pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	i2c_pins.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF4;
	i2c_pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	i2c_pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	i2c_pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//PB6 ---> I2C_SCL
	i2c_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	//Init this pin
	GPIO_Init(&i2c_pins);

	//PB7 ---> I2C_SDA
	i2c_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
	//Init this pin
	GPIO_Init(&i2c_pins);
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


void I2C1_Inits(void)
{
	// Choose the peripheral
	I2C1Handle.pI2Cx = I2C1;

	// Configure I2C1 peripheral
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;

	// Init the I2C1
	I2C_Init(&I2C1Handle);


}

void delay(void)
{
	for(uint32_t i = 0; i < (500000/2); i++);
}



int main(void)
{
	// the command code
	uint8_t command_code;

	// the received data length
	uint8_t recived_len;


	//semi-hosting code
	initialise_monitor_handles();
	printf("the application is running\n");


	// Button Configuration
	GPIO_ButtonInits();

	// I2C pins configurations
	I2C1_GPIOInits();


	// I2C peripheral Initialization
	I2C1_Inits();

	// Enable the peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	// Enabling ACK after enabling the peripheral
	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	while(1)
	{
		// wait until the button is pressed
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0) )
		// some delay to avoid debouncing
		delay();

		// the first command code related to data length
		command_code = 0x51;


		//Send command to get the data length
		I2C_MasterSendData(&I2C1Handle, &command_code, 1, SLAVE_ADDR, I2C_ENABLE_SR);


		//Get the data received length
		I2C_MasterReceiveData(&I2C1Handle, &recived_len, 1, SLAVE_ADDR, I2C_ENABLE_SR);


		// the second command code related to send the actual data
		command_code = 0x52;

		//send the command
		I2C_MasterSendData(&I2C1Handle, &command_code, 1, SLAVE_ADDR, I2C_ENABLE_SR);

		// Receive the actual data
		I2C_MasterReceiveData(&I2C1Handle, received_buffer, recived_len, SLAVE_ADDR, I2C_DISABLE_SR);

		//adding null character
		received_buffer[recived_len+1]= '\0';

		//printing out the data
		printf("Data: %s",received_buffer);

	}


	return 0;
}
