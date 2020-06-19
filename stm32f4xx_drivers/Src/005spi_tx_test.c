/*
 * 005spi_tx_test.c
 *
 *  Created on: May 24, 2020
 *      Author: Hamzy
 */
#include <string.h>
#include "stm32f407xx.h"


/*  Pin Configurations
 * PB12 ---> SPI2_NSS
 * PB13 ---> SPI2_SCK
 * PB14 ---> SPI2_MISO
 * PB15 ---> SPI2_MOSI
 * ALT Function = 5
 */

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t spi_pin;

	spi_pin.pGPIOx = GPIOB;
	spi_pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	spi_pin.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF5;
	spi_pin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	spi_pin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	spi_pin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SPI2_NSS
	//spi_pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	//GPIO_Init(&spi_pin);


	// SPI2_SCK
	spi_pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&spi_pin);

	// SPI2_MISO
	//spi_pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	//GPIO_Init(&spi_pin);

	// SPI2_MOSI
	spi_pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&spi_pin);
}


void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;

	//1. SPI base address
	SPI2handle.pSPIx = SPI2;

	SPI2handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPI_Config.SPI_SCLKSpeed = SPI_SCLK_SPEED_DIV2; // SCLK = 8 MHz
	SPI2handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPI_Config.SPI_CPOL = SPI_CPOL_HIGH;
	SPI2handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPI_Config.SPI_SSM = SPI_SMM_EN; //Software slave select enable



	//3. Enable SP2
	SPI_Init(&SPI2handle);
}

int main(void)
{

	char user_data[] = "Hello World";

	// Initialize the GPIO pins as SPI
	SPI2_GPIOInits();

	// Initialize the SPIs
	SPI2_Inits();

	//Keeping the NSS pin high to avoid MODF
	SPI_SSIConfig(SPI2, ENABLE);

	//disable multi master mode
	//SPI_SSOEConfig(SPI2, ENABLE);

	//2. Enable the SPI peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	// sending data
	SPI_SendData(SPI2, (uint8_t *) user_data, strlen(user_data));


	//Infinite loop
	while(1)
	{

	}


	return 0;
}
