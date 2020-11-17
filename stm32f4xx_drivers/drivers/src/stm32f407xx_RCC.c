/*
 * stm32f407xx_RCC.c
 *
 *  Created on: July 24, 2020
 *      Author: Hamzy
 */

#include "stm32f407xx_RCC.h"



uint16_t AHB_Prescaler[AHB_PRESCALE] = {2, 4, 8, 16, 64, 128, 256, 512};
uint16_t APB1_Prescaler[APB1_PRESCALE] = {2, 4, 8, 16};


/*************************** Peripheral Clock Setup ****************************
    * @func   - RCC_GetPCLK_1Value
	* @brief  - Return the APB1 clock value
	* @param  - void :
	* @return - clock value e.g. 16,000,000 16 MHz etc.
	* @Note   - Non
***************************************************************************/
uint32_t RCC_GetPCLK_1Value(void)
{
	// 1. Get / Check waht is our clock source
	uint8_t clockSource, AHB_Factor, APB1_Factor;
	uint32_t SystemClock = 0, PeripheralClock;

	uint8_t temp;


	// Get CFGR second bit value then mask with 0b00000011
	clockSource = ( (RCC->CFGR >>RCC_CFGR_SWS) & 0x3);

	// if SWS = 0 Then System clock = HSI 16 MHz
	if (clockSource == HSI_SOURCE)
	{
		SystemClock = HSI;
	}
	else if(clockSource == HSE_SOURCE)
	{
		SystemClock = HSE;
	}
	else if(clockSource == PLL_SOURCE)
	{
		SystemClock = RCC_GetPLLOutputClock();
	}


	// for AHB prescaler
	temp = ( (RCC->CFGR >> RCC_CFGR_HPRE) & 0xf);

	// If temp < 8 no presecaler used
	if (temp < 8)
	{
		//if HPRE < 8 then AHB prescaller = 1 No prescalling
		AHB_Factor = AHB_Factor_BY_1;
	}
	else
	{
		// E.G. if temp = 9
		// AHB_prescaler [9-8] = 4
		AHB_Factor = AHB_Prescaler[temp-8];
	}

	// for APB1
	// Get the value of PPRE1
	temp = ( (RCC->CFGR >> RCC_CFGR_PPRE1) & 0x7 );
	if (temp < 4)
	{
		APB1_Factor = APB1_Factor_BY_1;
	}
	else
	{
		APB1_Factor = APB1_Prescaler[temp-4];
	}

	// Calculate the peripgheral clock = SystemClock (HSI)/ AHBPrescaller / APB1Prescaller
	PeripheralClock = (SystemClock / AHB_Factor) / APB1_Factor;

	return PeripheralClock;
}



/*************************** Peripheral Clock Setup ****************************
    * @func   - RCC_GetPCLK_2Value
	* @brief  - Return the APB2 clock value
	* @param  - void :
	* @return - clock value e.g. 16,000,000 16 MHz etc.
	* @Note   - Non
***************************************************************************/
uint32_t RCC_GetPCLK_2Value(void)
{
	uint32_t systemClock = 0, temp, pclk2;
	uint8_t clockSource = (RCC->CFGR >> 2) & 0x3;
	uint8_t AHB_Factor, APB2_Factor;

	if (clockSource == RESET)
	{
		systemClock = 16000000;
	}
	else
	{
		systemClock = 8000000;
	}
	temp = (RCC->CFGR >> RCC_CFGR_HPRE) & 0xF;

	if (temp < 0x08)
	{
		AHB_Factor = SET;
	}
	else
	{
		AHB_Factor = AHB_Prescaler[temp-8];
	}



	temp = (RCC->CFGR >> RCC_CFGR_PPRE2) & 0x7;

	if(temp < 0x04)
	{
		APB2_Factor = SET;
	}
	else
	{
		APB2_Factor = APB1_Prescaler[temp-4];
	}
	pclk2 = (systemClock / AHB_Factor) / APB2_Factor;

	return pclk2;
}





/*************************** Get the actual PLL clock value ****************************
    * @func   - RCC_GetPLLOutputClock
	* @brief  - Return the PLL clock value
	* @param  - void :
	* @return - clock value e.g. 16,000,000 16 MHz etc.
	* @Note   - Non
***************************************************************************/
uint32_t RCC_GetPLLOutputClock(void)
{
	return 0;
}
