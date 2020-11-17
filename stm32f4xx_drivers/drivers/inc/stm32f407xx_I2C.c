/*
 * stm32f407xx_I2C.c
 *
 *  Created on: Jul 20, 2020
 *      Author: Hamzy
 */


#include "stm32f407xx_I2C.h"

/*********** Helper Private Functions*************/
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);



/************************** Generating Start condition ***********************
    * @func   - I2C_GenerateStartCondition
	* @brief  - Generate start codition
	* @param  - *pI2Cx: pointer I2C base address
	* @return - None
	* @Note   - None
***************************************************************************/
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	// Set I2Cx CR1 bith START
	pI2Cx->CR1 |= (SET<<I2C_CR1_START);
}


/************************** Send Address phase & write ***********************
    * @func   - I2C_ExecuteAddressPhaseWrite
	* @brief  - Send Address phase & write
	* @param  - *pI2Cx: pointer I2C base address
	* @param  - SlaveAddr: the address of the slave
	* @return - None
	* @Note   - None
***************************************************************************/
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	// 1. Shift the slave address to make space for the read / write bit
	SlaveAddr = SlaveAddr << 1;

	// 2. clearing the 0 bit to send write
	SlaveAddr &= ~(1);

	// 3. Sending the address & r/w bit
	pI2Cx->DR = SlaveAddr;
}


/************************** Send Address phase & read ***********************
    * @func   - I2C_ExecuteAddressPhaseRead
	* @brief  - Send Address phase & read
	* @param  - *pI2Cx: pointer I2C base address
	* @param  - SlaveAddr: the address of the slave
	* @return - None
	* @Note   - None
***************************************************************************/
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	// 1. Shift the slave address to make space for the read / write bit
	SlaveAddr = SlaveAddr << 1;

	// 2. Setting the 0 bit to send read
	SlaveAddr |= 1;

	// 3. Sending the address & r/w bit
	pI2Cx->DR = SlaveAddr;
}


/************************** Clearing Addr flag ***********************
    * @func   - I2C_ClearADDRFlag
	* @brief  - read SR1 & SR2 flags
	* @param  - *pI2CHandle: pointer I2C handle structure
	* @return - None
	* @Note   - None
***************************************************************************/
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummy_read;

	// check for device mode
	if ( pI2CHandle->pI2Cx->SR2 & (I2C_FLAG_MSL) )
	{
		// The device is in master mode
		// check for device state
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize == SET)
			{
				// first disable ack
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				//clear the ADDR flag (read SR1, read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void) dummy_read;
			}
		}
		else
		{
			// busy in Tx
			//clear the ADDR flag (read SR1, read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void) dummy_read;
		}
	}
	else
	{
		// device is in slave mode
		//clear the ADDR flag (read SR1, read SR2)
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void) dummy_read;
	}
}


/************************** Generating Stop condition ***********************
    * @func   - I2C_GenerateStopCondition
	* @brief  - Generate Stop codition
	* @param  - *pI2Cx: pointer I2C base address
	* @return - None
	* @Note   - None
***************************************************************************/
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( SET << I2C_CR1_STOP);
}


/************************** Sending data from an interrupt ***********************
    * @func   - I2C_MasterHandleTXEInterrupt
	* @brief  - load Tx buffer content into DR from an interrupt
	* @param  - *pI2CHandle: pointer I2C handle structure
	* @return - None
	* @Note   - None
***************************************************************************/
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if (pI2CHandle->TxLen > RESET)
	{
		// load the DR with what the pointer points to
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		// Decrement the Tx length
		pI2CHandle->TxLen--;

		// Increment the pointer of the Tx buffer
		pI2CHandle->pTxBuffer++;
	}
}



/************************** receiving data from an interrupt ***********************
    * @func   - I2C_MasterHandleRXNEInterrupt
	* @brief  - load data to Rx buffer from the DR in an interrupt
	* @param  - *pI2CHandle: pointer I2C handle structure
	* @return - None
	* @Note   - None
***************************************************************************/
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	// in case of i byte is remaining
	if (pI2CHandle->RxSize == SET)
	{
		// Store the DR content inside the Rx buffer.
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;

		//Decrement the data length
		pI2CHandle->RxLen--;
	}

	//if there are more than 1 byte of data
	if ( pI2CHandle->RxSize > SET )
	{
		// if only 2 byte of data then
		if ( pI2CHandle->RxLen == 2 )
		{
			// disable acking so no more bytes will be received
			I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
		}

		// then for all read the DR
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;

		// Increment the Rx buffer
		pI2CHandle->pRxBuffer++;

		// Decrement the data length
		pI2CHandle->RxLen--;
	}

	// If there is no more data
	if ( pI2CHandle->RxLen == RESET )
	{
		// close the I2C data reception and notify the application

		// 1. if there is no Sr then generate stop cond.
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
		{
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		// 2. Close I2C Rx comm.
		I2C_CloseReceiveData(pI2CHandle);

		// 3. Notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}




/*******************************************************************************
 *                        APIs supported by this driver
 *       for more information about the APIs check the functions definitions
 *                           APIs Implementations
 *
 *******************************************************************************/

/*************************** Peripheral Clock Setup ****************************
    * @func   - I2C_PClkControl
	* @brief  - Enables or disables peripheral clock
	* @param  - *pI2Cx : I2C peripheral Base address
	* @param  - clkValue : ENABLE or DISABLE
	* @return - None
	* @Note   - None
	***************************************************************************/
void I2C_PClkControl(I2C_RegDef_t *pI2Cx, uint8_t clkValue)
{
	if (clkValue == ENABLE)
		{
			if (pI2Cx == I2C1)
			{
				I2C1_PCLK_EN();
			}
			else if (pI2Cx == I2C2)
			{
				I2C2_PCLK_EN();
			}
			else if (pI2Cx == I2C3)
			{
				I2C3_PCLK_EN();
			}
		}
		else
		{
			if (pI2Cx == I2C1)
			{
				I2C1_PCLK_DI();
			}
			else if (pI2Cx == I2C2)
			{
				I2C2_PCLK_DI();
			}
			else if (pI2Cx == I2C3)
			{
				I2C3_PCLK_DI();
			}
		}
}




/*********************** Peripheral De-initialization ***************************
    * @func   - I2C_DeInit
	* @brief  - Deinitialize the I2Cx peripheral
	* @param  - *pI2Cx : I2C Peripheral Base address
	* @return - None
	* @Note   - None
	***************************************************************************/
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if (pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}
	else if (pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}
	else if (pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
}



/*********************** I2C Peripheral Control ***************************
    * @func   - I2C_PeripheralControl
	* @brief  - Enabling / Disabling the I2C peripheral
	* @param  - *pI2Cx : I2C Peripheral Base address
	* @param  - SPIStatus: ENABLE / DISABLE
	* @Note   - None
***************************************************************************/

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t Status)
{
	if (Status == ENABLE)
	{
		pI2Cx->CR1 |= (1<<I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &= ~(1<<I2C_CR1_PE);
	}
}


/*************************** Peripheral Initialization ****************************
    * @func   - I2C_Init
	* @brief  - Initialize the I2Cx peripheral
	* @param  - *pSPIHandle : pointer I2C handle structure
	* @return - None
	* @Note   - None
***************************************************************************/
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	// 1. Configure the mode (FM / SM)
	// 2. Configured the speed of SCL
	// 3. Configure the device address (slave mode)
	// 4. Enable ACK (disable by default in STM32F4)
	// 5. Configure the rise time for I2C pins

	// 1. First and Foremost Enable I2Cx clock
	I2C_PClkControl(pI2CHandle->pI2Cx, ENABLE);


	// 2. Let's start with CR1 register
	uint32_t tempreg = 0;

	tempreg |= (pI2CHandle->I2C_Config.I2C_AckControl << I2C_CR1_ACK);

	// Assign the handle value to the actual I2C CR1 register
	pI2CHandle->pI2Cx->CR1 = tempreg;

	// 3. Then go to CR2 register
	// FREQ bits which depend on APB1 clock which depend on clock source
	// Clock source by default "HSI"
	tempreg = 0;
	tempreg |= RCC_GetPCLK_1Value() / 1000000U;

	// Assign this value to FREQ of CR2
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3f);


	// 4. Configure OAR1 register
	tempreg = 0; //////////////////////////
	tempreg |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1); // <<1 as it starts from 1 : 8 bit

	/*** the 14 bit has to be kept 1 by software***/
	tempreg |= (1<<14);

	pI2CHandle->pI2Cx->OAR1 = tempreg;

	// 5. CCR calculation
	uint16_t ccr_reg = 0;
	tempreg = 0;

	// Get speed from user
	// if speed <= standard mode 100000
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//Configure standard mode CCR 15th bit
		// Calculate CCR = Tscl / 2 * Tpclk == fpclk / 2 * fscl
		ccr_reg = (RCC_GetPCLK_1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );


		tempreg |= (ccr_reg & 0xfff); // mask first 12 bits
	}
	else
	{
		// mode is fast mode configure the 15th bit
		tempreg |= (1<<I2C_CCR_FS);

		// Assign the duty cycle
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);

		//check the entered Duty Cycle
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			// Calculate CCR = Tscl / 3 * Tpclk == fpclk / 3 * fscl
			ccr_reg = (RCC_GetPCLK_1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		}
		else
		{
			// Calculate CCR = Tscl / 25 * Tpclk == fpclk / 25* fscl
			ccr_reg = (RCC_GetPCLK_1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		}


		tempreg |= (ccr_reg & 0xfff); // mask first 12 bits
	}

	// Assing the tempreg value to the actual CCR
	pI2CHandle->pI2Cx->CCR = tempreg;



	// 5. TRISE configuration
	// if speed <= standard mode 100000
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
		{

		 // device mode == standard mode
		 // TRISE = max TRISE * fpclk
		 // in SM Trise (max) = 1000 ns = 1Ms = 10^-6
		 // Tris = fpclk / 1000,000;
		tempreg = (RCC_GetPCLK_1Value() / 1000000U) + 1;
		}
		else
		{
			// the mode is Fast mode FM
			// in FM mode Trise = 300 ns = .3 Ms
			// Trise = f[clk * 300 / 1000000000
			tempreg = ( (RCC_GetPCLK_1Value() * 300) / 1000000000U) + 1;
		}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3f);
}


/*********************** I2C Get a flag status ***************************
    * @func   - I2C_GetFlagStatus
	* @brief  - check if a flag is set or reset
	* @param  - *pI2Cx : I2C Peripheral Base address
	* @param  - FlagName : value @I2C_FLAG
	* @return - Flage Status values FLAG_SET / FLAG_RESET
	* @Note   - None
	***************************************************************************/
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


/*************************** Master Sending Data ****************************
    * @func   - I2C_MasterSendData
	* @brief  - Sending data @ master mode
	* @param  - *pSPIHandle: pointer I2C handle structure
	* @param  - *pTxbuffer:  pointer to Tx buffer -shift register-
	* @param  - DataLenght:  data length -supposedly 1 byte-
	* @param  - SlaveAddr:   address of the concerned slave
	* @param  - Sr: Repeating start values @I2C_Sr
	* @return - None
	* @Note   - None
***************************************************************************/
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t DataLenght, uint8_t SlaveAddr,uint8_t Sr)
{
	// 1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);


	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
		while( ! I2C_GetFlagStatus( pI2CHandle->pI2Cx, I2C_FLAG_SB) );


	//3. Send the address of the slave with r/w bit set to w(0) (total 8 bits )
		I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);


	//4. Confirm that address phase is completed by checking the ADDR flag in teh SR1
		while( ! I2C_GetFlagStatus( pI2CHandle->pI2Cx, I2C_FLAG_ADDR) );


	//5. clear the ADDR flag according to its software sequence
	//   Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
		I2C_ClearADDRFlag(pI2CHandle);

	//6. send the data until DataLenght becomes 0
		while(DataLenght > RESET)
		{
			// check if the TXE is set?
			while( ! I2C_GetFlagStatus( pI2CHandle->pI2Cx, I2C_FLAG_TXE) );

			// Load the Tx buffer to the DR register
			pI2CHandle->pI2Cx->DR = *pTxbuffer;

			// Increment the Tx buffer (incrementing the pointer itself)
			pTxbuffer++;

			// Decrement the data length
			DataLenght--;
		}

		//7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
		//   Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
		//   when BTF=1 SCL will be stretched (pulled to LOW)
		while( ! I2C_GetFlagStatus( pI2CHandle->pI2Cx, I2C_FLAG_TXE) );
		while( ! I2C_GetFlagStatus( pI2CHandle->pI2Cx, I2C_FLAG_BTF) );


		//8. Generate STOP condition and master need not to wait for the completion of stop condition.
		//   Note: generating STOP, automatically clears the BTF
		if(Sr == I2C_DISABLE_SR)
		{
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}
}


/*************************** Enable / Disable Ack ****************************
    * @func   - I2C_ManageAcking
	* @brief  - Enable or Disable ACK
	* @param  - *pI2Cx: pointer I2C base address
	* @param  - Status: Values @I2C_AckControl
	* @return - None
	* @Note   - None
***************************************************************************/
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t Status)
{
	// Enable the ACK
	if(Status == I2C_ACK_ENABLE)
	{
		pI2Cx->CR1 |= (SET << I2C_CR1_ACK);
	}
	else
	{
		// Disable the ACK
		pI2Cx->CR1 &= ~(SET << I2C_CR1_ACK);
	}
}


/*************************** Master Receiving Data ****************************
    * @func   - I2C_MasterReceiveData
	* @brief  - Receiving data @ master mode
	* @param  - *pSPIHandle:pointer I2C handle structure
	* @param  - *pRxbuffer: pointer to Rx buffer -shift register-
	* @param  - DataLenght: data length -supposedly 1 byte-
	* @param  - SlaveAddr: address of the concerned slave
	* @param  - Sr: Repeating start values @I2C_Sr
	* @return - None
	* @Note   - None
***************************************************************************/
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t DataLenght, uint8_t SlaveAddr,uint8_t Sr)
{
	// 1. Generate start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. Confirm the start condition is completed by checking SB flag is set in SR1
    // note: until SB is set the SCL will be stretched to low
	while ( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB) );

	// 3. Send address phase = Slave address + 1 for read
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	// 4. Wait until the address phase is completed by checking the ADDR flag in SR1
	while ( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR) );


	// Procedures in case of reading 1 byte of data
	if (DataLenght == 1)
	{
		// 1. Disable ACKing
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		// 2. Clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		// 3. Wait until RxNE is SET
		while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		// 4. Once the data arrived to he shift register
		// Generate Stop condition
		if (Sr == I2C_DISABLE_SR)
		{
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		// 5. Read the data in Rx buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}

	// Procedures in case of reading more than 1 byte of data
		if (DataLenght > 1)
		{
		   // 1. Clear ADDR flag
		   I2C_ClearADDRFlag(pI2CHandle);

		   // 2. Read the data until DataLength becomes = 0;
		   for(uint32_t i = DataLenght; i > 0; i--)
		   {
			   // 3. Wait until RxNE is SET
			   while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE) );

			   // if the data is just 2 bytes remaining
			   if(i == 2)
			   {
				   // Disable ACKing
				   I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				   // Once the data arrived to he shift register
				   	// Generate Stop condition
				   		if (Sr == I2C_DISABLE_SR)
				   		{
				   			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				   		}
			   }
			   // Read the data in Rx buffer
			   *pRxBuffer = pI2CHandle->pI2Cx->DR;

			   // increment the pRxBuffer
			   pRxBuffer++;
		   }
		}
		// If I was Enabling the ACK then re-enable it again
		if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
		{
			I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
		}
}



/*********************** I2C Send Data Interrupt Mode ***************************
    * @func   - I2C_MasterSendDataIT
	* @brief  - Sending the data interrupt mode
	* @param  - *I2C_Handle_t: I2C handle structure
	* @param  - *TxBuffer: pointer to data inside Tx buffer
	* @param  - DataLength: Number of bits to be transmitted
	* @param  - SlaveAddr: The address of Slave
	* @param  - Sr: Repeated start values @I2C_Sr
	* @return - state: values @I2CState
	* @Note   - None
***************************************************************************/
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t DataLenght, uint8_t SlaveAddr, uint8_t Sr)
{
	// Store the current / actual state of the I2C in this local variable
	uint8_t busyState = pI2CHandle->TxRxState;

	// if the communication is not busy neither in Tx nor Rx
	if ( (busyState != I2C_BUSY_IN_TX) && (busyState != I2C_BUSY_IN_RX) )
	{
		// Store the input arguments data in the I2C handle members
		pI2CHandle->pTxBuffer = pTxbuffer;
		pI2CHandle->TxLen = DataLenght;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX; // make the comm. busy in transmitting (Tx state)
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		// Generate a start condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// Enable buffer interrupt enable bit (ITBUFEN)
		// if TX == 1 or RX == 1 ---> interrupt is fired
		pI2CHandle->pI2Cx->CR2 |= (SET << I2C_CR2_ITBUFEN);


		// Enable event interrupt enable bit (ITEVTEN)
		// if SB == 1 or ADDR == 1 or STOP == 1, etc---> interrupt is fired
		pI2CHandle->pI2Cx->CR2 |= (SET << I2C_CR2_ITEVTEN);


		// Enable error interrupt enable bit (ITERREN:)
		// if ARLO == 1 or AF == 1 or OVR == 1, etc---> interrupt is fired
		pI2CHandle->pI2Cx->CR2 |= (SET << I2C_CR2_ITERREN);

	}
	return busyState;
}


/*********************** I2C Receive Data Interrupt Mode ***************************
    * @func   - I2C_MasterReceiveDataIT
	* @brief  - Sending the data interrupt mode
	* @param  - *I2C_Handle_t: I2C handle structure
	* @param  - *RxBuffer: pointer to data inside Rx buffer
	* @param  - DataLength: Number of bits to be transmitted
	* @param  - SlaveAddr: The address of Slave
	* @param  - Sr: Repeated start values @I2C_Sr
	* @return - state: values @I2CState
	* @Note   - None
***************************************************************************/
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t DataLenght, uint8_t SlaveAddr,uint8_t Sr)
{
	// Store the current / actual state of the I2C in this local variable
	uint8_t busyState = pI2CHandle->TxRxState;

	// if the communication is not busy neither in Tx nor Rx
	if ( (busyState != I2C_BUSY_IN_TX) && (busyState != I2C_BUSY_IN_RX) )
	{
		// Store the input arguments data in the I2C handle members
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = DataLenght;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX; // make the comm. busy in receiving (Rx state)
		pI2CHandle->RxSize = DataLenght;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		// Generate a start condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// Enable buffer interrupt enable bit (ITBUFEN)
		// if TX == 1 or RX == 1 ---> interrupt is fired
		pI2CHandle->pI2Cx->CR2 |= (SET << I2C_CR2_ITBUFEN);


		// Enable event interrupt enable bit (ITEVTEN)
		// if SB == 1 or ADDR == 1 or STOP == 1, etc---> interrupt is fired
		pI2CHandle->pI2Cx->CR2 |= (SET << I2C_CR2_ITEVTEN);


		// Enable error interrupt enable bit (ITERREN:)
		// if ARLO == 1 or AF == 1 or OVR == 1, etc---> interrupt is fired
		pI2CHandle->pI2Cx->CR2 |= (SET << I2C_CR2_ITERREN);

	}
	return busyState;
}


/*************************** Configure an Interrupt *************************
    * @func   - I2C_IRQInterruptConfig
	* @brief  - Set Interrupt : Enabling the interrupt of the mentioned IRQ number
	* @param  - IRQNumber : which Interrupt will be enabled ----> @IRQn
	* @param  - IRQState : Enable / Disable the Interrupt
	* @return - void
	* @Note   - None
	***************************************************************************/
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQState)
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
    * @func   - I2C_IRQPriorityConfig
	* @brief  - Setting the priority for a given Interrupt
	* @param  - IRQNumber : which Interrupt will be enabled @IRQn
	* @param  - IRQPriority : Set the Interrupt Priority    @IRQPriority
	* @return - void
	* @Note   - None
	***************************************************************************/
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
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


/*************************** Serving the I2C Event ISR *************************
    * @func   - I2C_EV_IRQHandling
	* @brief  - What to do when I2C event interrupt happens
	* @param  - pI2CHandle: pointer to I2C handle structure
	* @return - void
	* @Note   - None
	***************************************************************************/
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	// Interrupt handling for both master & slave mode of a device
	uint32_t temp1, temp2, temp3;

	//let's check if the interrupt enable bit are set or not
	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	// let's check is SB flag is set or not
	temp3 = pI2CHandle->pI2Cx->SR1 & (I2C_FLAG_SB);


	// sum up all the event that may generate interrupt

	// 1. Handle for Interrupt generated by SB
	// Note: SB flag is only applicable in Master mode
	if (temp1 && temp3)
		{
			// SB flag is set == interrupt generated because of SB flag
			// Let's implement address phase r/w

			//let's check the communicatIon busy state
				if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
				{
					I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
				}

				else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
				{
					I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
				}
		}

	// let's check is ADDR flag is set or not
	temp3 = pI2CHandle->pI2Cx->SR1 & (I2C_FLAG_ADDR);

	// 2. Handle for interrupt generated by ADDR event
	// Note: in Master mode address is sent
	//       in Slave mode address is matched
	if (temp1 && temp3)
		{
			// ADDR flag is set
			I2C_ClearADDRFlag(pI2CHandle);
		}


	// let's check is BTF flag is set or not
	temp3 = pI2CHandle->pI2Cx->SR1 & (I2C_FLAG_BTF);

	// 3. Handle for interrupt generated by BTF (Byte transfer finished) event
	if (temp1 && temp3)
		{
			// BTF flag is set
			// Check if the communication is busy in transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				// Make sure that the TxE is also set
				if( pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE) )
				{
					// Here both BTF & TxE ==1
					// if all data is sent (DataLength == 0)
					if (pI2CHandle->TxLen == RESET)
					{
						//Close the communication
						// 1. Generate stop condition, only if repeated start is disabled
						if(pI2CHandle->Sr == I2C_DISABLE_SR)
						{
							I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
						}
						// 2. Reset all members of the I2C handle structure
						I2C_CloseSendData(pI2CHandle);

						//3. Notify the application about transmission complete.
						I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
					}

				}
			}
			else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				// We don't close the comm. like this when receiving
			}
		}

	// let's check is STOPF flag is set or not
	temp3 = pI2CHandle->pI2Cx->SR1 & (I2C_FLAG_STOPF);

	// 4. Handle for interrupt generated by STOPF event
	// Note: STOPF flag is set only in Slave mode, in Master mode it is never set
	if (temp1 && temp3)
		{
			// STOPF flag is set
			// Clear the STOPF by read SR1 then write to CR1
			// reading SR1 already done above in temp3
			pI2CHandle->pI2Cx->CR1 |= 0x0000;

			// Notify the application that STOP is detected
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);

		}


	// let's check is TxE flag is set or not
	temp3 = pI2CHandle->pI2Cx->SR1 & (I2C_FLAG_TXE);

	// 5. Handle for interrupt generated by TxE event
	if (temp1 && temp2 && temp3)
		{
			// TxE flag is set
			// check the device mode
			if ( pI2CHandle->pI2Cx->SR2 & (I2C_FLAG_MSL) )
			{
				// Check if the communication is busy in transmission
				if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
				{
					I2C_MasterHandleTXEInterrupt(pI2CHandle);
				}
			}
			else
			{
				// Device ins slave mode
				// make sure the slave is really transmitter mode
				if( pI2CHandle->pI2Cx->SR2 & (I2C_FLAG_TRA) )
				{
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
				}
			}
		}

	// let's check is RxNE flag is set or not
	temp3 = pI2CHandle->pI2Cx->SR1 & (I2C_FLAG_RXNE);

	// 6. Handle for interrupt generated by RxNE event
	if (temp1 && temp2 && temp3)
		{
			// RxNE flag is set
			// check device mode
			if ( pI2CHandle->pI2Cx->SR2 & (I2C_FLAG_MSL) )
			{
				// device in master mode
				// check if the device is busy receiving
				if ( pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
				{
					I2C_MasterHandleRXNEInterrupt(pI2CHandle);
				}
			}
			else
			{
				// device in slave mode
				// make sure the slave is really receiving
				if ( !(pI2CHandle->pI2Cx->SR2 & (I2C_FLAG_TRA)) )
				{
					// Notify the application of the data received
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
				}
			}
		}
}

/*************************** Serving the I2C Error ISR *************************
    * @func   - I2C_ER_IRQHandling
	* @brief  - What to do when I2C error interrupt happens
	* @param  - pI2CHandle: pointer to I2C handle structure
	* @return - void
	* @Note   - None
***************************************************************************/
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1, temp2;

	// check the value of ITERREN bit in CR2 (SET/ RESET)
	temp2 = (pI2CHandle->pI2Cx->CR2) & (SET << I2C_CR2_ITERREN);

	// 1. Check for bus error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (I2C_FLAG_BERR);

	//  if both are set
	if (temp1 && temp2)
	{
		// Clear the BERR bit by writing 0 to it
		pI2CHandle->pI2Cx->SR1 &= ~(I2C_FLAG_BERR);

		// Notify the application with the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}

	// 2. check for arbitration lost error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (I2C_FLAG_ARLO);

	//  if both are set
		if (temp1 && temp2)
		{
			// Clear the ARLO bit
			pI2CHandle->pI2Cx->SR1 &= ~(I2C_FLAG_ARLO);

			// Notify the application with the ARLO error
			I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
		}

	// 3. check for ACK failure error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (I2C_FLAG_AF);

	//  if both are set
		if (temp1 && temp2)
		{
			// Clear the AF bit
			pI2CHandle->pI2Cx->SR1 &= ~(I2C_FLAG_AF);

		    // Notify the application with the AF error
			I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
		}

	// 4. Check for Overrun / Underrun error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (I2C_FLAG_OVR);

	//  if both are set
		if (temp1 && temp2)
		{
			// Clear the OVR bit
			pI2CHandle->pI2Cx->SR1 &= ~(I2C_FLAG_OVR);

			// Notify the application with the OVR error
			I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
		}

	// 4. Check for time out error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (I2C_FLAG_TIMEOUT);

	//  if both are set
	    if (temp1 && temp2)
	    {
		    // Clear the OVR bit
			pI2CHandle->pI2Cx->SR1 &= ~(I2C_FLAG_TIMEOUT);

			// Notify the application with the OVR error
			I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
		}

}


/******************* Close I2C transmission communication ******************
    * @func   - I2C_CloseSendData
	* @brief  - Reseting all I2C handle structure members
	* @param  - pI2CHandle: pointer to I2C handle structure
	* @return - void
	* @Note   - None
************************************************************************/
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	// 1. Disable ITBUFEN
	pI2CHandle->pI2Cx->CR2 &= ~(SET << I2C_CR2_ITBUFEN);

	// 2. Disable ITEVTEN
	pI2CHandle->pI2Cx->CR2 &= ~(SET << I2C_CR2_ITEVTEN);

	// 4. make state ready
	pI2CHandle->TxRxState = I2C_READY;

	// 4. Reset Tx buffer
	pI2CHandle->pTxBuffer = NULL;

	// 5. Reset Tx buffer length
	pI2CHandle->TxLen = RESET;
}



/******************* Close I2C reception communication ******************
    * @func   - I2C_CloseReceiveData
	* @brief  - Reseting all I2C handle structure members
	* @param  - pI2CHandle: pointer to I2C handle structure
	* @return - void
	* @Note   - None
************************************************************************/
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	// 1. Disable ITBUFEN
	pI2CHandle->pI2Cx->CR2 &= ~(SET << I2C_CR2_ITBUFEN);

	// 2. Disable ITEVTEN
	pI2CHandle->pI2Cx->CR2 &= ~(SET << I2C_CR2_ITEVTEN);

	// 3. make state ready
	pI2CHandle->TxRxState = I2C_READY;

	// 4. Reset Rx buffer
	pI2CHandle->pRxBuffer = NULL;

	// 5. Reset Rx buffer length
	pI2CHandle->RxLen = RESET;

	// 6. Reset Rx buffer size
	pI2CHandle->RxSize = RESET;

	// Enable Ack again
	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}



/*************************** Slave Sending Data ****************************
    * @func   - I2C_SlaveSendData
	* @brief  - Sending data @ slave mode
	* @param  - *pI2C: pointer I2C base address structure
	* @param  - data: data being sent
	* @return - None
	* @Note   - None
***************************************************************************/
void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data)
{
	// Load the data into DR register
	pI2C->DR = data;
}


uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
	// return the received data
	return (uint8_t) pI2C->DR;
}



/*************************** Enable interrupt at slave mode ****************************
    * @func   - I2C_SlaveEnableDisableCallbackEvents
	* @brief  - Enable & setting interrupt control bits @ slave mode
	* @param  - *pI2C: pointer I2C base address structure
	* @param  - Status: Enable / Disable
	* @return - None
	* @Note   - None
***************************************************************************/
void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t Status)
{
	if (Status == ENABLE)
	{
		// enable CR2 interrupt control bits
		pI2Cx->CR2 |= (SET << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 |= (SET << I2C_CR2_ITERREN);
		pI2Cx->CR2 |= (SET << I2C_CR2_ITEVTEN);
	}
	else
	{
		// disable all CR2 interrupt control bits
		pI2Cx->CR2 &= ~(SET << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 &= ~(SET << I2C_CR2_ITERREN);
		pI2Cx->CR2 &= ~(SET << I2C_CR2_ITEVTEN);
	}
}
