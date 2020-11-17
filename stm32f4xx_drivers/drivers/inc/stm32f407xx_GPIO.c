/*
 * stm32f407xx_GPIO.c
 *
 *  Created on: Mar 24, 2020
 *      Author: Hamzy
 */


#include "stm32f407xx_GPIO.h"


/*******************************************************************************
 *                        APIs supported by this driver
 *       for more information about the APIs check the functions definitions
 *                           APIs Implementations
 *
 *******************************************************************************/

/*************************** Peripheral Clock Setup ****************************
    * @func   - GPIO_PClkControl
	* @brief  - Enables or disables peripheral clock
	* @param  - *GPIOx : GPIO Port Base address
	* @param  - clkValue : ENABLE or DISABLE
	* @return - None
	* @Note   - None
	***************************************************************************/
void GPIO_PClkControl(GPIO_RegDef_t *pGPIOx, uint8_t clkValue)
{
	if (clkValue == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
	}
	else
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
	}
}


/*************************** Peripheral Initialization ****************************
    * @func   - GPIO_Init
	* @brief  - Initialize the GPIO peripheral
	* @param  - *pGPIOHandle : pointer GPIO handle structure
	* @return - None
	* @Note   - None
	***************************************************************************/

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{

	/*********** Enable the peripheral clock***************/
	GPIO_PClkControl(pGPIOHandle->pGPIOx, ENABLE);


	uint32_t temp = 0;              //temporary register


	/****** 1. configure a GPIO pin mode********/
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// none interrupt mode
		// temp = selected pin mode << 2 * pin number (0 = input , 1 = output  , 2 , etc...)
		// temp = 1 (output mode) << 2 * 2 (pin_2) = 0b0000 0000 0000 0000 0000 0000 0010 0000
		temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );

		// store temp into the actual register
		// when dealing with hardware register always and only use |= or &=
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) ); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp;                                                   //setting
	}
	else
	{
		/**********interrupt mode***************/
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INT_FT)
		{
			//1. Enable FTSR
			EXTI->EXTI_FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//2. Clear RTSR to ensure that only falling edge enabled
			EXTI->EXTI_RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INT_RT)
		{
			//1. Enable RTSR
			EXTI->EXTI_RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//2. Clear FTSR to ensure that only rising edge enabled
			EXTI->EXTI_FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INT_FRT)
		{
			//1. Enable both register FTSR & RTSR
			EXTI->EXTI_RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->EXTI_FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//3. Select the Port from SYSCFG_EXTICR
		uint8_t temp3, temp4;

		uint8_t PortCode;

		/* for example button on PC13
		 * temp3= 13/4 = 3 ---> go to EXTICR[3] which contain EXTI13*/
		temp3 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) / 4;

		/* for example button on PC13
		 * temp3= (13%4) *4 = 1 * 4  ---> go to EXTICR[13] in the bit no.4*/
		temp4 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) % 4;

		PortCode = GPIO_PORT_EXTI_CODE(pGPIOHandle->pGPIOx);

		// Enable the SYSCFG clock
		SYSCFG_PCLK_EN();

		// Enable the EXTICR register
		SYSCFG->EXTICR[temp3] |= PortCode << (temp4 * 4);




		//4. Enable EXTI interrupt delivery by IMR (interrupt mask register)
		EXTI->EXTI_IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp = 0;



	/********* 2. configure a GPIO pin speed**************/

	// temp = selected pin speed << 2 * pin number (0 = low , 1 = medium , 2 = fast , etc...)
	// temp = 1 (output speed) << 2 * 2 (pin_2) = 0b0000 0000 0000 0000 0000 0000 0010 0000
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );

	// store temp into the actual register
	// when dealing with hardware register always and only use |= or &=

	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) ); //clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;                                                     //setting

	temp = 0;




	/********** 3. configure a GPIO pin output type *************/

	// temp = selected pin output type  << pin number (0 = push/pull , 1 = open drain)
	// temp = 1 (output speed) << 2 (pin_2) = 0b0000 0000 0000 0100
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );

	// store temp into the actual register
	// when dealing with hardware register always and only use |= or &=
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) ); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;                                                 //setting

	temp = 0;




	/********** 4. configure a GPIO pin pupd setting **********/
	// temp = selected pin pull up/down  << 2 * pin number (0 = no pull up/down , 1 = pull up, 2 = pull down)
	// temp = 1 (pull down) << 2 * 2 (pin_2) = 0b0000 0000 0000 0000 0000 0000 0010 0000
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );

	// store temp into the actual register
	// when dealing with hardware register always and only use |= or &=
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) ); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;                                                      //setting


	temp = 0;


	/************ 5. configure a GPIO pin alternate functionality *************/
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		// create 2 temp variables
		uint8_t temp1, temp2;

		// to get each register of AFR[0] or AFR[1]
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;

		// to get each bit inside the register
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2) ); //clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= ( pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2) );
	}

}


/*********************** Peripheral De-initialization ***************************
    * @func   - GPIO_Init
	* @brief  - Deinitialize the GPIO peripheral
	* @param  - *GPIOx : GPIO Port Base address
	* @return - None
	* @Note   - None
	***************************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
}



/************************ Read from GPIO Input Pin ***************************
    * @func   - GPIO_ReadFromInputPin
	* @brief  - Read a GPIO pin value
	* @param  - *GPIOx : GPIO Port Base address
	* @param  - PinNumber : pin number
	* @return - uint8_t pin value = SET/ RESET
	* @Note   - None
	***************************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t PinValue;

	/* Ex: PinNumber == 5
	 * PinValue = pGPIOx->IDR >> 5 == 00000001
	 * PinValue &= 0x00000001
	 * type casted to uint8_t  */

	PinValue = (uint8_t)( (pGPIOx->IDR >> PinNumber) & 0x00000001 );

	return PinValue;
}

/*************************** Read from GPIO Input Port *************************
    * @func   - GPIO_ReadFromInputPort
	* @brief  - Read a GPIO port value
	* @param  - *GPIOx : GPIO Port Base address
	* @return - uint16_t port value = PortSET (0xFFFF) : RESET (0x0000)
	* @Note   - None
	***************************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t PortValue;

	PortValue = (uint16_t) (pGPIOx->IDR);
	return PortValue;
}


/*************************** Write to GPIO Output Pin *************************
    * @func   - GPIO_WriteToOutputPin
	* @brief  - Write a GPIO pin value
	* @param  - *GPIOx : GPIO Port Base address
	* @param  - PinNumber : pin number
	* @param - uint8_t pin value = SET/RESET
	* @return - void
	* @Note   - None
	***************************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t PinVlaue)
{
	// if pin value = 1
	if (PinVlaue == GPIO_PIN_SET)
	{
	// Set the corresponding bit field in the ODR
	pGPIOx->ODR |= (1 << PinNumber);
	}
	// if pin value = 0
	else
	{
		// Reset the corresponding bit field in the ODR
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}


/*************************** Write to GPIO Output Port *************************
    * @func   - GPIO_WriteToOutputPort
	* @brief  - Write a GPIO port value
	* @param  - *GPIOx : GPIO Port Base address
	* @param  - uint16_t port value = PortSET (0xFFFF) : RESET (0x0000)
	* @return - void
	* @Note   - None
	***************************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t PortValue)
{
	pGPIOx->ODR = PortValue;

}



/*************************** Toggle a GPIO Pin Value *************************
    * @func   - GPIO_ToggleOutputPin
	* @brief  - Write a GPIO port value
	* @param  - *GPIOx : GPIO Port Base address
	* @param  - PinNumber : pin number
	* @return - void
	* @Note   - None
	***************************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1<<PinNumber);
}



/*************************** Configure an Interrupt *************************
    * @func   - GPIO_IRQConfig
	* @brief  - Set Interrupt : IRQ number ,priority, Enabling
	* @param  - IRQNumber : which Interrupt will be enabled ----> @IRQn
	* @param  - IRQState : Enable / Disable the Interrupt
	* @return - void
	* @Note   - None
	***************************************************************************/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQState)
{
	if (IRQState == ENABLE)
	{
		if (IRQNumber < 32)
		{
			//program ISER0 register
			NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if ( (IRQNumber >= 32 ) && (IRQNumber < 64) )
		{
			//program ISER1 register
			NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if ( (IRQNumber >= 64 ) && (IRQNumber < 96) )
		{
			// program ISER2 register
			NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
	}
	else //IRQ State == DISABLE
	{
		if (IRQNumber < 32)
		{
			//program ICER0 register
			NVIC_ICER0 |= (1 << IRQNumber);

		}
		else if ( (IRQNumber >= 32 ) && (IRQNumber < 64) )
		{
			//program ICER1 register
			NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if ( (IRQNumber >= 64 ) && (IRQNumber < 96) )
		{
			//program ICER2 register
			NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
		}
	}

}



/*************************** Serving the ISR *************************
    * @func   - GPIO_IRQPriorityConfig
	* @brief  - Setting the priority for a given Interrupt
	* @param  - IRQNumber : which Interrupt will be enabled @IRQn
	* @param  - IRQPriority : Set the Interrupt Priority    @IRQPriority
	* @return - void
	* @Note   - None
	***************************************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// 1. First find out the IPRx and section required
	uint8_t iprx = IRQNumber / 4;            // Get the number of the IPR register
	uint8_t iprxSection = IRQNumber % 4;     // Get the number of the section inside the IPRx register

	// 2. Get the number of bits the priority shall be shifted @prioritybits
	uint8_t shiftAmount = (iprxSection * 8) + (8 - NO_OF_PRIORITY_BITS);  // iprsection * 8 as each 8 bit for IRQ
	                                                                      // 4 bits of these 8 are reserved

	// 3. Setting the priority on the section of the IPRx
	*( NVIC_IPR_BASEADDR + iprx ) |= ( IRQPriority << shiftAmount );

}





/*************************** Serving the ISR *************************
    * @func   - GPIO_IRQHandling
	* @brief  - What to do when the interrupt happens
	* @param  - PinNumber : which pin issued the interrupt
	* @return - void
	* @Note   - None
	***************************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{
	// clear  the pending register PR corresponding to the pin number
	if (EXTI->EXTI_PR & (1 << PinNumber)) // if the bit is set
	{
		// clear it by writing 1 to it
		EXTI->EXTI_PR |= (1 << PinNumber);
	}
}

