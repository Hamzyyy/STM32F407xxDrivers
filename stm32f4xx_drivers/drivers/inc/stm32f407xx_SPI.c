/*
 * stm32f407xx_SPI.c
 *
 *  Created on: Apr 26, 2020
 *      Author: Hamzy
 */

#include "stm32f407xx_SPI.h"

/** Some Helper Functions **/
// make it static in order to be private / local it can not be called in the application
static void SPI_TXE_Interrupt_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_RXNE_Interrupt_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_OVR_Iterrupt_Handle(SPI_Handle_t *pSPIHandle);


/*******************************************************************************
 *                        APIs supported by this driver
 *       for more information about the APIs check the functions definitions
 *                           APIs Implementations
 *
 *******************************************************************************/

/*************************** Peripheral Clock Setup ****************************
    * @func   - SPI_PClkControl
	* @brief  - Enables or disables peripheral clock
	* @param  - *SPIx : SPI peripheral Base address
	* @param  - clkValue : ENABLE or DISABLE
	* @return - None
	* @Note   - None
	***************************************************************************/
void SPI_PClkControl(SPI_RegDef_t *pSPIx, uint8_t clkValue)
{
	if (clkValue == ENABLE)
	{
		if (pSPIx == SPI1)
		{
		  SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}
	else
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
	}
}

/*************************** Peripheral Initialization ****************************
    * @func   - SPI_Init
	* @brief  - Initialize the SPIx peripheral
	* @param  - *pSPIHandle : pointer SPI handle structure
	* @return - None
	* @Note   - None
	***************************************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{

	/******* Enabling the peripheral clock ********/
	SPI_PClkControl(pSPIHandle->pSPIx , ENABLE);


	// First let's configure CR1 register
	uint32_t temp = 0; // to be store all the values derived from user @ SPI_Handle_t then pass it to the CR1 register


	// 1. Configure the Device Mode
	temp |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;

	// 2. Configure the BusConfig
	// 2.1. If the Bus is full duplex
	if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// 2.1.1. the BIDIMODE bit shall be cleared
		temp &= ~(1 << SPI_CR1_BIDIMODE); // BIDIMODE == 15 bit
	}
	//2.2.If the Bus is half duplex
	else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// 2.1.1. the BIDIMODE bit shall be set
		temp |= (1 << SPI_CR1_BIDIMODE); // BIDIMODE == 15 bit
	}
	//2.3 if the bus is full duplex but receive only
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//2.3.1. the BIDIMODE bit shall be cleared
		temp &= ~(1 << SPI_CR1_BIDIMODE); // BIDIMODE == 15 bit

		//2.3.2. the RXONLY bit shall be set
		temp |= (1 << SPI_CR1_RXONLY); // RXONLY == 10 bit
	}


	// 3. Configure SPI serial clock speed
	temp |= pSPIHandle->SPI_Config.SPI_SCLKSpeed << SPI_CR1_BR; // baud rate bit are 3:5

	// 4. Configure the data frame format
	temp |= pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF; // DFF bit == 11

	// 5.Configure clock phase
	temp |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA; // CPHA bit == 0

	// 6. Configure clock polarity
	temp |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL; // CPOL bit == 1

	//7. Slave Select
	temp |= pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM;

	//7. Assign the temp value to the actual CR1 register
	pSPIHandle->pSPIx->CR1 = temp;



}

/*********************** Peripheral De-initialization ***************************
    * @func   - SPI_Init
	* @brief  - Deinitialize the SPI peripheral
	* @param  - *SPIx : SPI Peripheral Base address
	* @return - None
	* @Note   - None
	***************************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if (pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if (pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if (pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
	else if (pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}
}


/*********************** SPI Send Data ***************************
    * @func   - SPI_SendData
	* @brief  - Sending the data
	* @param  - *SPIx : SPI Peripheral Base address
	* @param  - *TxBuffer : pointer to data inside Tx buffer
	* @param  - DataLength : Number of bits to be transmitted
	* @return - None
	* @Note   - None
	***************************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t DataLenght)
{
	// checking the length
	while(DataLenght > 0)
	{
		// 1. Wait until the Tx buffer is empty
		// as long as TXE == 0 keep waiting
		while (SPI_GetSPIstatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		// 2. check the DFF bit in the CR1
		if ( (pSPIx->CR1 & (1 << SPI_CR1_DFF) ) )
		{
			// the data frame format is 16-bits
			// Load the data in the DR
			pSPIx->DR = *( (uint16_t*) pTxBuffer); // type cast the pointer to 16 bit then dereference

			// decrement the data length twice
			DataLenght--;
			DataLenght--;

			// increment Tx pointer to points to the next data
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			// the data frame format is 8-bits
			// Load the data in the DR
			pSPIx->DR = *pTxBuffer; // dereference

			// decrement the data length twice
			DataLenght--;

			// increment Tx pointer to points to the next data
			pTxBuffer ++;
		}
	}
}

/*********************** SPI Get a flag status ***************************
    * @func   - SPI_GetSPIstatus
	* @brief  - check if a flag is set or reset
	* @param  - *SPIx : SPI Peripheral Base address
	* @param  - FlagName : value @SPI_FLAG
	* @return - Flage Status values FLAG_SET / FLAG_RESER
	* @Note   - None
	***************************************************************************/
uint8_t SPI_GetSPIstatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*********************** SPI Peripheral Control ***************************
    * @func   - SPI_PControl
	* @brief  - Enabling / Disabling the SPI peripheral
	* @param  - *SPIx : SPI Peripheral Base address
	* @param  - SPIStatus: ENABLE / DISABLE
	* @Note   - None
***************************************************************************/

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t Status)
{
	if (Status == ENABLE)
	{
	    pSPIx->CR1 |= (1<<SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SPE);
	}
}

/*********************** SPI SSOE Bit Configuration ***************************
    * @func   - SPI_SSOEConfig
	* @brief  - Enabling / Disabling the multi-master communication
	* @param  - *SPIx : SPI Peripheral Base address
	* @param  - Status: ENABLE / DISABLE
	* @Note   - None
***************************************************************************/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t Status)
{
	if (Status == ENABLE)
	{
		pSPIx->CR2 |= (1<<SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1<<SPI_CR2_SSOE);
	}
}


/*********************** SPI SSI Bit Configuration ***************************
    * @func   - SPI_SSIConfig
	* @brief  - Setting or Clearing the value of NSS pin by SW
	* @param  - *SPIx : SPI Peripheral Base address
	* @param  - Status: ENABLE / DISABLE
	* @Note   - None
***************************************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t Status)
{
	if (Status == ENABLE)
	{
		pSPIx->CR1 |= (1<<SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SSI);
	}
}




/*********************** SPI Receive Data ***************************
    * @func   - SPI_ReceiveData
	* @brief  - Receiving the data
	* @param  - *SPIx : SPI Peripheral Base address
	* @param  - *RxBuffer : pointer to data inside Rx buffer
	* @param  - DataLength : Number of bits to be transmitted
	* @return - None
	* @Note   - None
***************************************************************************/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t DataLenght)
{
	// checking the length
		while(DataLenght > 0)
		{
			// 1. Wait until the RXNE buffer is empty
			// as long as RXNE == 0 keep waiting
			while (SPI_GetSPIstatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

			// 2. check the DFF bit in the CR1
			if ( (pSPIx->CR1 & (1 << SPI_CR1_DFF) ) )
			{
				// the data frame format is 16-bits
				// Load the data from the DR to the RX buffer
				*( (uint16_t*) pRxBuffer) = pSPIx->DR; // type cast the pointer to 16 bit then dereference

				// decrement the data length twice
				DataLenght--;
				DataLenght--;

				// increment Rx pointer to points to the next data
				(uint16_t*)pRxBuffer++;
			}
			else
			{
				// the data frame format is 8-bits
				// Load the data from the DR to the Rx buffer
				*pRxBuffer = pSPIx->DR; // dereference

				// decrement the data length twice
				DataLenght--;

				// increment Tx pointer to points to the next data
				pRxBuffer ++;
			}
		}
}




/*********************** SPI Verify the response ***************************
    * @func   - SPI_VerifyResponse
	* @brief  - Acknowledgement of receiving from the slave
	* @param  - ackbyte : ACK OR NACK (0 / 1)
	* @return - 0 / 1
	* @Note   - None
***************************************************************************/
uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
	if(ackbyte == ENABLE)
	{
		//ack
		return ENABLE;
	}

	return DISABLE;
}



/*************************** Configure an Interrupt *************************
    * @func   - SPI_IRQInterruptConfig
	* @brief  - Set Interrupt : Enabling the interrupt of the mentioned IRQ number
	* @param  - IRQNumber : which Interrupt will be enabled ----> @IRQn
	* @param  - IRQState : Enable / Disable the Interrupt
	* @return - void
	* @Note   - None
	***************************************************************************/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQState)
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



/*************************** Setting the priority of the interrupt *************************
    * @func   - SPI_IRQPriorityConfig
	* @brief  - Setting the priority for a given Interrupt
	* @param  - IRQNumber : which Interrupt will be enabled @IRQn
	* @param  - IRQPriority : Set the Interrupt Priority    @IRQPriority
	* @return - void
	* @Note   - None
	***************************************************************************/
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
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


/*********************** SPI Send Data Interrupt Mode ***************************
    * @func   - SPI_SendDataIT
	* @brief  - Sending the data interrupt mode
	* @param  - *pSPIHandle : SPI handle structure
	* @param  - *TxBuffer : pointer to data inside Tx buffer
	* @param  - DataLength : Number of bits to be transmitted
	* @return - state : SPI_BUSY_IN_TX or SPI_READY
	* @Note   - None
***************************************************************************/
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t DataLenght)
{
	// Check if the SPI is busy in TX
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		// 1. Save Tx buffer address & Data length information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = DataLenght;

		// 2. Mark the SPI state as busy in transmission so that
		// no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState =SPI_BUSY_IN_TX;

		// 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_TXEIE);
	}
	// 4. Data transmission will be handled by ISR code


	return state;
}



/*********************** SPI Receive Data Interrupt Mode ***************************
    * @func   - SPI_ReceiveDataIT
	* @brief  - Receiving the data interrupt mode
	* @param  - *pSPIHandle : SPI handle structure
	* @param  - *TxBuffer : pointer to data inside Tx buffer
	* @param  - DataLength : Number of bits to be transmitted
	* @return - state : SPI_BUSY_IN_RX or SPI_READY
	* @Note   - None
***************************************************************************/
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t DataLenght)
{
	// Check if the SPI is busy in RX
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		// 1. Save Rx buffer address & Data length information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = DataLenght;

		// 2. Mark the SPI state as busy in receiving so that
		// no other code can take over same SPI peripheral until receiving is over
		pSPIHandle->RxState =SPI_BUSY_IN_RX;

		// 3. Enable the RXNEIE control bit to get interrupt whenever RXNE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_RXNEIE);
	}
	// 4. Data transmission will be handled by ISR code


	return state;
}



/*************************** Serving the SPI ISR *************************
    * @func   - SPI_IRQHandling
	* @brief  - What to do when SPI interrupt happens
	* @param  - pSPIHandle: pointer to SPI handle structure
	* @return - void
	* @Note   - None
	***************************************************************************/
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	// temporary variables
	uint8_t temp1, temp2;

	// 1. lets check the TXE flag

	// if the TXE flag is set
	temp1 = pSPIHandle->pSPIx->SR & (1<<SPI_SR_TXE);

	// if the TXEIE is set (TXE interrupt enabled)
	temp2 = pSPIHandle->pSPIx->CR2 & (1<<SPI_CR2_TXEIE);

	if (temp1 && temp2)
	{
		// Handle TXE interrupt
		SPI_TXE_Interrupt_Handle(pSPIHandle);
	}


	// 2. lets check the RXNE flag

	// if the RXNE flag is set
	temp1 = pSPIHandle->pSPIx->SR & (1<<SPI_SR_RXNE);

	// if the RXNEIE is set (RXNE interrupt enabled)
	temp2 = pSPIHandle->pSPIx->CR2 & (1<<SPI_CR2_RXNEIE);

	if (temp1 && temp2)
	{
		// Handle RNXE interrupt
		SPI_RXNE_Interrupt_Handle(pSPIHandle);
	}



	// 3. lets check the OVR flag

	// if the OVR flag is set
	temp1 = pSPIHandle->pSPIx->SR & (1<<SPI_SR_OVR);

	// if the ERRIE is set (ERRIE interrupt enabled)
	temp2 = pSPIHandle->pSPIx->CR2 & (1<<SPI_CR2_ERRIE);

	if (temp1 && temp2)
	{
		// Handle OVR interrupt
		SPI_OVR_Iterrupt_Handle(pSPIHandle);
	}

}



/*************************** Clearing the Overrun *************************
    * @func   - SPI_ClearOVRFlag
	* @brief  - Clear the OVR flag in the application
	* @param  - pSPIx : base address of SPIx
	* @return - void
	* @Note   - None
	***************************************************************************/
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;

	// 1 read the data register DR
	temp = pSPIx->DR;
	// 2 read from the status register SR
	temp = pSPIx->SR;

	//type cast to void unless will give you error "not used"
	(void) temp;
}


/*************************** Close the Transmission *************************
    * @func   - SPI_CloseTransmission
	* @brief  - Closing the transmission process abruptly
	* @param  - pSPIHandle : SPI handle struct
	* @return - void
	* @Note   - None
	***************************************************************************/
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	// RESET TXEIE, disable TX interrupt
	pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_TXEIE);

	// RESET also all the SPI handle variables
	pSPIHandle->pTxBuffer = NULL;

	pSPIHandle->TxLen = RESET;

	pSPIHandle->TxState = SPI_READY;
}


/*************************** Close the Reception *************************
    * @func   - SPI_CloseReception
	* @brief  - Closing the reception process abruptly
	* @param  - pSPIHandle : SPI handle struct
	* @return - void
	* @Note   - None
	***************************************************************************/
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	        // RESET TXEIE
			pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_RXNEIE);

			// RESET also all the SPI handle variables
			pSPIHandle->pRxBuffer = NULL;

			pSPIHandle->RxLen = RESET;

			pSPIHandle->RxState = SPI_READY;

}



/********** Helper functions Implementations *********/

// 1. TXE Interrupt handling function
static void SPI_TXE_Interrupt_Handle(SPI_Handle_t *pSPIHandle)
{
		// 1. check the DFF bit in the CR1
		if ( (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF) ) )
		{
		    // the data frame format is 16-bits
			// Load the data in the DR
			pSPIHandle->pSPIx->DR = *( (uint16_t*) pSPIHandle->pTxBuffer); // type cast the pointer to 16 bit then dereference

			// decrement the data length twice
			pSPIHandle->TxLen--;
			pSPIHandle->TxLen--;

			// increment Tx pointer to points to the next data
			(uint16_t*)pSPIHandle->pTxBuffer++;
		}
		else
		{
			// the data frame format is 8-bits
			// Load the data in the DR
			pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer; // dereference

			// decrement the data length twice
			pSPIHandle->TxLen--;

			// increment Tx pointer to points to the next data
			pSPIHandle->pTxBuffer++;
		}
		if ( ! pSPIHandle->TxLen )
		{
			// when TxLen = 0; close the SPI transmission
			SPI_CloseTransmission(pSPIHandle);


			// A callback function that calls a function from application
			SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_TX_CMPLT);
		}
}



// 2. RXNE Interrupt handling function
static void SPI_RXNE_Interrupt_Handle(SPI_Handle_t *pSPIHandle)
{
	// 1. check the DFF bit in the CR1
	if ( (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF) ) )
	{
		// the data frame format is 16-bits
		// Load the data from the DR
		*( (uint16_t*) pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR; // type cast the pointer to 16 bit then dereference

		// decrement the data length twice
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;

		// increment Rx pointer to points to the next data
		(uint16_t*)pSPIHandle->pRxBuffer++;
	}
	else
	{
		// the data frame format is 8-bits
		// Load the data from the DR
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR; // dereference

		// decrement the data length twice
		pSPIHandle->RxLen--;

		// increment Rx pointer to points to the next data
		pSPIHandle->pRxBuffer++;
	}
	if ( ! pSPIHandle->RxLen )
	{
		// when TxLen = 0; close the SPI transmission
		SPI_CloseReception(pSPIHandle);


		// A callback function that calls a function from application
		SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}



// 3. OVR Interrupt handling function
static void SPI_OVR_Iterrupt_Handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	// 1. clear the OVR flag
	if (pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		//1.1 read the data register DR
		temp = pSPIHandle->pSPIx->DR;
		// 1.2 read from the status register SR
		temp = pSPIHandle->pSPIx->SR;
	}

	// type cast to void unless it will give you warning "Not used"
	(void) temp;
	// 2. inform the application
	SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_OVR_ERR);

}



__attribute__((weak)) void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHandle, uint8_t AppEvent)
{
	//This is a weak implementation. the user application may override this function.
}

