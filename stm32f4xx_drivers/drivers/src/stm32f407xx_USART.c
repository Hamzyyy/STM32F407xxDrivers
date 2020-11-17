/*
 * stm32f407xx_USART.C
 *
 *  Created on: Oct 14, 2020
 *      Author: Hamzy
 */

#include "stm32f407xx_USART.h"






/*************************** Setting the baud rate value *************************
    * @func   - USART_SetBaudRate
	* @brief  - Setting the baud rate for a USART peripheral
	* @param  - *pI2Cx : pointer to USARTx Peripheral Base address
	* @param  - BaudRate :the desired baud rate value  @USART_Baud
	* @return - void
	* @Note   - None
	***************************************************************************/
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
	//Variable to hold the APB clock
	uint32_t PCLKx;

	// devition factor
	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

	 uint32_t tempreg = RESET;

	 //Get the value of APB bus clock in to the variable PCLKx

	 if(pUSARTx == USART1 || pUSARTx == USART6)
	  {
		   //USART1 and USART6 are hanging on APB2 bus
		   PCLKx = RCC_GetPCLK_2Value();
	  }
	 else
	 {
		 PCLKx = RCC_GetPCLK_1Value();
	 }

	 //Check for OVER8 configuration bit
	 if(pUSARTx->CR1 & (SET << USART_CR1_OVER8))
	 {
		 usartdiv = ( (25 * PCLKx) / (2 * BaudRate) );
	 }
	 else
	 {
		 usartdiv = ( (25 * PCLKx) / (4 * BaudRate) );
	 }

	 //Calculate the Mantissa part
	 M_part = usartdiv / 100;

	 //Place the Mantissa part in appropriate bit position . refer USART_BRR
	 tempreg |= M_part << 4;


	  //Extract the fraction part
	  F_part = ( usartdiv - (M_part * 100) );

	  //Calculate the final fractional
	  if(pUSARTx->CR1 & (SET << USART_CR1_OVER8))
	  {
		  //OVER8 = 1 , over sampling by 8
		  // 1. multiply * 8, then + 50 to round / ceiling
		  // 2. divide / 100 to return to its actual value
		  // 3. then mask to clear any undesired numbers
		  F_part = ( ( (F_part * 8) + 50 ) / 100 ) & ( (uint8_t) 0x07 );
	  }
	  else
	  {
		  F_part = ( ( (F_part * 16) + 50 ) / 100 ) & ( (uint8_t) 0x0F );
	  }

	  //Place the fractional part in appropriate bit position . refer USART_BRR
	  tempreg |= F_part;

	  //copy the value of tempreg in to BRR register
	  pUSARTx->BRR = tempreg;

}







/*************************** Peripheral Initialization ****************************
    * @func   - USART_Init
	* @brief  - Initialize the USARTx peripheral
	* @param  - *pUSARTHandle : pointer USART handle structure
	* @return - None
	* @Note   - None
***************************************************************************/
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	// temporary register
	uint32_t tempreg = 0;

	// Enable the peripheral clock
	USART_PClkControl(pUSARTHandle->pUSARTx, ENABLE);

	/****** the configuration of CR1 register ******/
	// enable the mode as entered in handle structure
	if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		// Mode Tx ---> Enable TE bit only
		tempreg |= (SET << USART_CR1_TE);
	}
	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		// Mode Rx ---> Enable RE bit only
		tempreg |= (SET << USART_CR1_RE);
	}
	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TX_RX)
	{
		// Mode Tx & Rx ---> Enable TE & RE bits
		tempreg |= (SET << USART_CR1_TE) | (SET << USART_CR1_RE);
	}

	// configure the word length
	tempreg |= (pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M);

	// configure the parity control bit
	//1. if even parity
	if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN )
	{
		// Enable parity and leave the PS bit = 0
		tempreg |= (SET << USART_CR1_PCE);
	}
	else if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD)
	{
		// Enable parity bit and set the PS bit = 1
		tempreg |= (SET << USART_CR1_PCE);
		tempreg |= (SET << USART_CR1_PS);
	}

	// Load these values to the actual CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;

	/****** the configuration of CR2 register ******/
	tempreg = 0;
	// Configure the number of stop bits based on what entered
	// in handle structure
	tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

	// Load the value to the actual CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;


	/****** the configuration of CR3 register ******/
	tempreg = 0;

	// Configure the USART flow control
	if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		// Enable the CTSE bit
		tempreg |= (SET << USART_CR3_CTSE);
	}
	else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		// Enable the RTSE bit
		tempreg |= (SET << USART_CR3_RTSE);
	}
	else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		// Enable the CTSE bit
		tempreg |= (SET << USART_CR3_CTSE);
		// Enable the RTSE bit
		tempreg |= (SET << USART_CR3_RTSE);
	}

	// load the values to the actual CR3 register
	pUSARTHandle->pUSARTx->CR3 = tempreg;

	/****** the configuration of BRR Baud rate register ******/
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}


/*********************** Peripheral De-initialization ***************************
    * @func   - USART_DeInit
	* @brief  - Deinitialize the USARTx peripheral
	* @param  - *pI2Cx : USARTx Peripheral Base address
	* @return - None
	* @Note   - None
	***************************************************************************/
void USART_DeInit(USART_RegDef_t *pUSARTx)
{
	if (pUSARTx == USART1)
	{
		USART1_REG_RESET();
	}
	else if (pUSARTx == USART2)
	{
		USART2_REG_RESET();
	}
	else if (pUSARTx == USART3)
	{
		USART3_REG_RESET();
	}
}





/*********************** USART Peripheral Control ***************************
    * @func   - USART_PeripheralControl
	* @brief  - Enabling / Disabling the USART peripheral
	* @param  - *pUSARTx : USART Peripheral Base address
	* @param  - SPIStatus: ENABLE / DISABLE
	* @Note   - None
***************************************************************************/
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t Status)
{
	if (Status == ENABLE)
	{
		pUSARTx->CR1 |= (SET<<USART_CR1_UE);
	}
	else
	{
		pUSARTx->CR1 &= ~(SET<<USART_CR1_UE);
	}
}


/*************************** Peripheral Clock Setup ****************************
    * @func   - USART_PClkControl
	* @brief  - Enables or disables peripheral clock
	* @param  - *pUSARTx : USART peripheral Base address
	* @param  - clkValue : ENABLE or DISABLE
	* @return - None
	* @Note   - None
	***************************************************************************/
void USART_PClkControl(USART_RegDef_t *pUSARTx, uint8_t clkValue)
{
	if (clkValue == ENABLE)
	{
		if (pUSARTx == USART1)
		{
		  USART1_PCLK_EN();
		}
		else if (pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}
		else if (pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}
		else if (pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}
	}
	else
	{
		if (pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}
		else if (pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}
		else if (pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}
		else if (pUSARTx == UART4)
		{
			UART4_PCLK_DI();
		}
	}
}


/*********************** USART Get a flag status ***************************
    * @func   - USART_GetFlagStatus
	* @brief  - check if a flag is set or reset
	* @param  - *pUSARTx : USART Peripheral Base address
	* @param  - FlagName : value @USART_FLAG
	* @return - Flage Status values FLAG_SET / FLAG_RESET
	* @Note   - None
	***************************************************************************/
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint8_t FlagName)
{
	if(pUSARTx->SR & FlagName)
	{
		return FLAG_SET;
	}
	else
	{
		return FLAG_RESET;
	}
}



/************************** Clearing a USART flag ***********************
    * @func   - USART_ClearFlag
	* @brief  - read SR1 & SR2 flags
	* @param  - *pUSARTx : USART Peripheral Base address
	* @param  - FlagName : value @USART_FLAG
	* @return - None
	* @Note   - None
***************************************************************************/
void USART_ClearFlag(USART_RegDef_t *pUSARTx , uint16_t FlagName)
{
	pUSARTx->SR &= ~(FlagName);
}



/*************************** Sending Data ****************************
    * @func   - USART_SendData
	* @brief  - Sending data
	* @param  - *pUSARTHandle: pointer USART handle structure
	* @param  - *pTxbuffer:  pointer to Tx buffer -shift register-
	* @param  - DataLenght:  data length -supposedly 1 byte-
	* @return - None
	* @Note   - None
***************************************************************************/
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxbuffer, uint32_t DataLenght)
{
	// local buffer
	uint16_t *pData;
	// Loop of until Data Length number of bytes are transferred
	for (uint32_t i = RESET; i < DataLenght; i++)
	{
		// wait until the TxE flag is SET
		while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE) )
		{

		}
			//check if USART word length == 9bits
			if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
			{
				//if 9BIT load the DR with 2bytes, then masking the bits other than first 9 bits
				pData = (uint16_t *) pTxbuffer;

				// load the buffer into the actual DR register
				pUSARTHandle->pUSARTx->DR = ( *pData * (uint16_t) 0x01FF);

				if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
				{
					//No parity is used in this transfer , so 9bits of user data will be sent
					//Implement the code to increment pTxBuffer twice
					pTxbuffer++;
					pTxbuffer++;
				}
				else
				{
					//Parity bit is used in this transfer . so 8bits of user data will be sent
					//The 9th bit will be replaced by parity bit by the hardware
					pTxbuffer++;
				}
			}
			else
			{
				//This is 8bit data transfer
				pUSARTHandle->pUSARTx->DR = (*pTxbuffer  & (uint8_t)0xFF);

				//Implement the code to increment the buffer address
				pTxbuffer++;
			}
		}
	// wait until the TC flag is SET
	while ( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));
}



/*************************** Receiving Data ****************************
    * @func   - USART_ReceiveData
	* @brief  - Receiving data
	* @param  - *pUSARTHandle: pointer USART handle structure
	* @param  - *pRxbuffer: pointer to Rx buffer -shift register-
	* @param  - DataLenght: data length -supposedly 1 byte-
	* @return - None
	* @Note   - None
***************************************************************************/
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t DataLenght)
{
	// Loop of until Data Length number of bytes are transferred
		for (uint32_t i = RESET; i < DataLenght; i++)
		{
			// wait until the TxNE flag is SET
			while( !USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE) );

			//check if USART word length == 9bits
			if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
			{
				//We are going to receive 9bit data in a frame
				//Now, check are we using USART_ParityControl control or not
				if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
				{
					// No parity is used , so all 9bits will be of user data
				    //read only first 9 bits so mask the DR with 0x01FF
					*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t) 0x01FF);

					//Now increment the pRxBuffer two times
					pRxBuffer++;
					pRxBuffer++;
				}
				else
				{
					//Parity is used, so 8bits will be of user data and 1 bit is parity
					*pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
					 pRxBuffer++;
				}
			}
			else
			{
				//We are going to receive 8bit data in a frame
				//Now, check are we using USART_ParityControl control or not
				if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
				{
					//No parity is used , so all 8bits will be of user data
					//read 8 bits from DR
					*pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
				}
				else
				{
					//Parity is used, so , 7 bits will be of user data and 1 bit is parity
					//read only 7 bits , hence mask the DR with 0X7F
					*pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);
				}

				//Now , increment the pRxBuffer
				pRxBuffer++;
			}
		}
}

/*********************** USART Send Data Interrupt Mode ***************************
    * @func   - USART_SendDataIT
	* @brief  - Sending the data interrupt mode
	* @param  - *pUSARTHandle: USART handle structure
	* @param  - *TxBuffer: pointer to data inside Tx buffer
	* @param  - DataLength: Number of bits to be transmitted
	* @return - state: values @USARTState
	* @Note   - None
***************************************************************************/
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxbuffer, uint32_t DataLenght)
{
	// Store the current / actual state of the USART in this local variable
	uint8_t busyState = pUSARTHandle->TxState;

	// if the communication is not busy neither in Tx
	if (busyState != USART_BUSY_IN_TX)
	{
		// Store the input arguments data in the USART handle members
		pUSARTHandle->TxLen = DataLenght;
		pUSARTHandle->pTxBuffer = pTxbuffer;
		// make the comm. busy in transmitting (Tx state)
		pUSARTHandle->TxState = USART_BUSY_IN_TX;

		//Implement the code to enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= (SET << USART_CR1_TXEIE);

		//Implement the code to enable interrupt for TC
		pUSARTHandle->pUSARTx->CR1 |= ( SET << USART_CR1_TCIE);
	}
	return busyState;
}


/*********************** USART Receive Data Interrupt Mode ***************************
    * @func   - USART_ReceiveDataIT
	* @brief  - Sending the data interrupt mode
	* @param  - *pUSARTHandle: USART handle structure
	* @param  - *RxBuffer: pointer to data inside Rx buffer
	* @param  - DataLength: Number of bits to be transmitted
	* @return - state: values @USARTState
	* @Note   - None
***************************************************************************/
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t DataLenght)
{
	// Store the current / actual state of the USART in this local variable
	uint8_t busyState = pUSARTHandle->RxState;

	// if the communication is not busy neither in Rx
	if (busyState != USART_BUSY_IN_RX)
	{
		// Store the input arguments data in the USART handle members
		pUSARTHandle->RxLen = DataLenght;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		// make the comm. busy in transmitting (Rx state)
		pUSARTHandle->RxState = USART_BUSY_IN_RX;

		(void) pUSARTHandle->pUSARTx->DR; //??
		//Implement the code to enable interrupt for RNXE
		pUSARTHandle->pUSARTx->CR1 |= (SET << USART_CR1_RXNEIE);
	}
	return busyState;
}
/*************************** Configure an Interrupt *************************
    * @func   - USART_IRQInterruptConfig
	* @brief  - Set Interrupt : Enabling the interrupt of the mentioned IRQ number
	* @param  - IRQNumber : which Interrupt will be enabled ----> @IRQn
	* @param  - IRQState : Enable / Disable the Interrupt
	* @return - void
	* @Note   - None
	***************************************************************************/
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQState)
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
    * @func   - USART_IRQPriorityConfig
	* @brief  - Setting the priority for a given Interrupt
	* @param  - IRQNumber : which Interrupt will be enabled @IRQn
	* @param  - IRQPriority : Set the Interrupt Priority    @IRQPriority
	* @return - void
	* @Note   - None
	***************************************************************************/
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
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

/*************************** Serving the USART ISR *************************
    * @func   - USART_IRQHandling
	* @brief  - What to do when USART interrupt happens
	* @param  - pUSARTHandle: pointer to USART handle structure
	* @return - void
	* @Note   - None
	***************************************************************************/
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{
	// Interrupt handling for a device
	uint32_t temp1, temp2, temp3;
	uint16_t *pData;

	/************** Check for TC flag ****************/
	// let's check if TC flag is set or not
	temp1 = pUSARTHandle->pUSARTx->SR & (USART_FLAG_TC);

	// let's check if TCIE interrupt enable bit is set or not
	temp2 = pUSARTHandle->pUSARTx->CR1 & (SET << USART_CR1_TCIE);

	// 1. Handle for Interrupt generated by TC
	if (temp1 && temp2)
	{
		if (pUSARTHandle->TxState == USART_BUSY_IN_TX)
		{
			//Check the TxLen . If it is zero then close the data transmission
			if( ! pUSARTHandle->TxLen )
			{
				// Clear the TC flag
				pUSARTHandle->pUSARTx->SR &= ~(SET << USART_SR_TC);

				// Clear TCIE bit. to prevent more interrupt
				pUSARTHandle->pUSARTx->CR1 &= ~(SET << USART_CR1_TCIE);

				// make the comm. state is ready
				pUSARTHandle->TxState = USART_READY;

				// Reset Tx buffer
				pUSARTHandle->pTxBuffer = NULL;

				//Reset the Tx Length to zero ??
				pUSARTHandle->TxLen = RESET;

				//Call the application call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);
			}
		}
	}

	/************** Check for TXE flag ****************/
	// let's check if TxE flag is set or not
	temp1 = pUSARTHandle->pUSARTx->SR & (USART_FLAG_TXE);

	// let's check if TCIE interrupt enable bit is set or not
	temp2 = pUSARTHandle->pUSARTx->CR1 & (SET << USART_CR1_TXEIE);

	// 2. Handle for Interrupt generated by TXE
	if (temp1 && temp2)
	{
		if (pUSARTHandle->TxState == USART_BUSY_IN_TX)
		{
			//Check the TxLen . If it is zero then close the data transmission
			if( pUSARTHandle->TxLen > RESET )
			{
				// check if the word length is 9BITS or 8BITS
				if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//if 9BIT load the DR with 2bytes masking  the bits other than first 9 bits
					pData = (uint16_t *)pUSARTHandle->pTxBuffer;
					pUSARTHandle->pUSARTx->DR = (*pData & (uint16_t) 0x01FF);

					//check for USART_ParityControl
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used in this transfer , so 9bits of user data will be sent
						//Implement the code to increment pTxBuffer twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen -= 2;
					}
					else
					{
						//Parity bit is used in this transfer . so 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen -= SET;
					}
				}
				else
				{
					//This is 8bit data transfer
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer & (uint8_t) 0xFF);

					//Parity bit is used in this transfer . so 8bits of user data will be sent
					//The 9th bit will be replaced by parity bit by the hardware
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen -= SET;
				}
			}
			if (pUSARTHandle->TxLen == RESET)
			{
				//TxLen is zero
				//Implement the code to clear the TXEIE bit (disable interrupt for TXE flag)
				pUSARTHandle->pUSARTx->CR1 &= ~ (SET << USART_CR1_TXEIE);
			}
		}
	}

	/************** Check for RXNE flag ****************/
	// let's check if RxNE flag is set or not
	temp1 = pUSARTHandle->pUSARTx->SR & (USART_FLAG_RXNE);

	// let's check if TCIE interrupt enable bit is set or not
	temp2 = pUSARTHandle->pUSARTx->CR1 & (SET << USART_CR1_RXNEIE);

	// // 2. Handle for Interrupt generated by RxNE
	if (temp1 && temp2)
	{
		if (pUSARTHandle->RxState == USART_BUSY_IN_RX)
		{
			//Check the RxLen. If it is zero then close the data transmission
			if( pUSARTHandle->RxLen > RESET )
			{
				// check if the word length is 9BITS or 8BITS
				if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//We are going to receive 9bit data in a frame
					//Now, check are we using USART_ParityControl set or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used , so all 9bits will be of user data
						//read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

						//Now increment the pRxBuffer two times
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen -= 2;
					}
					else
					{
						//Parity is used, so 8bits will be of user data and 1 bit is parity
						*pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen -= SET;
					}
				}
				else
				{
					//We are going to receive 8bit data in a frame
					//Now, check are we using USART_ParityControl set or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						*pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t) 0xff);
					}
					else
					{
						//Parity is used, so , 7 bits will be of user data and 1 bit is parity
						//read only 7 bits , hence mask the DR with 0X7F
						*pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t) 0x7F);
					}
					//Now , increment the pRxBuffer
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->RxLen -= SET;
				}
			}
			if ( ! pUSARTHandle->RxLen)
			{
				// disable the RXNE
				// No more RxNE interrupt
				pUSARTHandle->pUSARTx->CR1 &= ~( SET << USART_CR1_RXNEIE );
				pUSARTHandle->RxState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_RX_CMPLT);
			}
		}
	}

	/************** Check for CTS flag ****************/
    // let's check if RxNE flag is set or not
	//Note : CTS feature is not applicable for UART4 and UART5


	// let's check if CTS flag is set or not
	temp1 = pUSARTHandle->pUSARTx->SR & (USART_FLAG_CTS);

    // let's check if CTSE enable bit is set or not
    temp2 = pUSARTHandle->pUSARTx->CR3 & (SET << USART_CR3_CTSE);

    // let's check if CTS interrupt enable bit is set or not
    // This bit is not available for UART4 & UART5
    temp3 = pUSARTHandle->pUSARTx->CR3 & (SET << USART_CR3_CTSIE);

    if (temp1 && temp2 & temp3)
    {
    	// Clear the CTS flag
    	pUSARTHandle->pUSARTx->SR &= ~(SET << USART_SR_CTS);

    	// Call the application callback
    	USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_CTS);
    }


    /************** Check for IDLE detection flag ****************/

    // let's check if CTS flag is set or not
    temp1 = pUSARTHandle->pUSARTx->SR & (USART_FLAG_IDLE);

    // let's check if CTSE enable bit is set or not
    temp2 = pUSARTHandle->pUSARTx->CR1 & (SET << USART_CR1_IDLEIE);

    if( temp1 && temp2)
    {
    	// Clear the IDLE flag
    	pUSARTHandle->pUSARTx->SR &= ~(SET << USART_SR_IDLE);

    	//this interrupt is because of idle
    	USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_IDLE);
    }



    /************** Check for Overrun detection flag ****************/

    // let's check if ORE flag is set or not
    temp1 = pUSARTHandle->pUSARTx->SR & (USART_FLAG_ORE);

	//Implement the code to check the status of RXNEIE  bit in the CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;

	if( temp1 && temp2)
	{
		//Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag .
        //this interrupt is because of Overrun error
	    USART_ApplicationEventCallback(pUSARTHandle, USART_ERROR_ORE);
	}

	/************** Check for Error detection flag ****************/

	//Noise Flag, Overrun error and Framing Error in multibuffer communication
	//We dont discuss multibuffer communication in this course. please refer to the RM
	//The blow code will get executed in only if multibuffer mode is used.

	// check if error interrupt enable is set or not
	temp2 = pUSARTHandle->pUSARTx->CR3 & (SET << USART_CR3_EIE);

	if(temp2)
	{
		// make temp1 only == SR, then shift per error type.
		temp1 = pUSARTHandle->pUSARTx->SR;

		//check for the FE flag (Framing Error)
		if (temp1 & USART_FLAG_FE)
		{
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERROR_FE);
		}

		//check for the NF flag (Noise Error)
		if (temp1 & USART_FLAG_NF)
		{
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERROR_NE);
		}

		//check for the ORE flag (Overrun Error)
		if(temp1 & USART_FLAG_ORE)
		{
			 USART_ApplicationEventCallback(pUSARTHandle, USART_ERROR_ORE);
		}

		//check for the PE flag (Parity Error)
		if(temp1 & USART_FLAG_PE)
		{
			 USART_ApplicationEventCallback(pUSARTHandle, USART_ERROR_PE);
		}
	}
}





__attribute__((weak)) void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t ApEv)
{

}
