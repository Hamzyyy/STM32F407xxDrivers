/*
 * stm32f407xx_SPI.h
 *
 *  Created on: Apr 25, 2020
 *      Author: Dell
 */

#ifndef INC_STM32F407XX_SPI_H_
#define INC_STM32F407XX_SPI_H_


#include <stdint.h>
#include "stm32f407xx.h"


/** Configuration Structure for a SPIx pin **/
typedef struct
{
	uint8_t SPI_DeviceMode;         /*!< Possible values from @SPI_DeviceMode >*/
	uint8_t SPI_BusConfig;           /*!< Possible values from @SPI_BusConfig >*/
	uint8_t SPI_SCLKSpeed;          /*!< Possible values from @@SPI_SclkSpeed >*/
	uint8_t SPI_DFF;                /*!< Possible values from @SPI_DFF >*/
	uint8_t SPI_CPOL;               /*!< Possible values from @SPI_ClkPol >*/
	uint8_t SPI_CPHA;               /*!< Possible values from @SPI_ClkPha >*/
	uint8_t SPI_SSM;                /*!< Possible values from @SPI_SSM >*/
}SPI_Config_t;


/** Handle Structure for a SPI pin **/
typedef struct
{
	SPI_RegDef_t *pSPIx;              // this hold the base address of the SPI the pin belongs to
	SPI_Config_t SPI_Config;          // this hold SPI configuration settings
}SPI_Handle_t;




/* @SPI_DeviceMode
 *  SPI DEVICE MODE
 */
#define SPI_DEVICE_MODE_SLAVE             0
#define SPI_DEVICE_MODE_MASTER            1


/* @SPI_BusConfig
 *  SPI BUS CONFIG
 */
#define SPI_BUS_CONFIG_FD                1   // full duplex
#define SPI_BUS_CONFIG_HD                2   // half duplex
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY    3



/* @SPI_SclkSpeed
 *  SPI Serial Speed
 */
#define SPI_SCLK_SPEED_DIV2              0
#define SPI_SCLK_SPEED_DIV4              1
#define SPI_SCLK_SPEED_DIV8              2
#define SPI_SCLK_SPEED_DIV16             3
#define SPI_SCLK_SPEED_DIV32             4
#define SPI_SCLK_SPEED_DIV64             5
#define SPI_SCLK_SPEED_DIV128            6
#define SPI_SCLK_SPEED_DIV256            7


/* @SPI_DFF
 * SPI Data Frame Format
 */
#define SPI_DFF_8BITS                    0
#define SPI_DFF_16BITS                   1



/* @SPI_ClkPol
 * SPI Serial Clock Polarity
 */
#define SPI_CPOL_LOW                    0  // Idle @ Low
#define SPI_CPOL_HIGH                   1  // Idle @ High



/*@SPI_ClkPha
 * SPI Serial Clock Phase
 */
#define SPI_CPHA_LOW                    0 // Slave Capture @ First Edge
#define SPI_CPHA_HIGH                   1 // Slave Capture @ Second Edge

/*@SPI_SSM
 * SPI Slave Select Management
 */
#define SPI_SMM_DI                      0 // Slave select @ hardware
#define SPI_SMM_EN                      1 // Slave Capture @ software

/*@SPI_FLAG
 * SPI related status flag definitions
 */
#define SPI_TXE_FLAG                    (1<<SPI_SR_TXE)   // Tx buffer empty
#define SPI_RXNE_FLAG                   (1<<SPI_SR_RXNE)  // RX buffer not empty
#define SPI_BUSY_FLAG                   (1<<SPI_SR_BSY)   // SPI busy


/*******************************************************************************
 *                        APIs supported by this driver
 *       for more information about the APIs check the functions definitions
 *
 *******************************************************************************/
/* Peripheral clock setup
 * Arguments == base address of SPIx channel, Value of clock (SET/RESET)
 * Return == void
 */
void SPI_PClkControl(SPI_RegDef_t *pSPIx, uint8_t clkValue);



/* SPIx Init/Dinit Functions
 * Arguments == pointer to handle structure
 * Return = void
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);  // there is a single register to shut down each peripheral clock RCC_APBxRSTR



/* SPIx Send data Functions
 * Arguments == pointer to register definition structure, pointer to Tx buffer, Data length
 * Return = void
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t DataLenght);



/* SPI Get flag status Functions
 * Arguments == pointer to register definition structure, flag name
 * Return = flag status SET / RESET
 */
uint8_t SPI_GetSPIstatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);



/* SPIx Receive data Functions
 * Arguments ==  pointer to register definition structure, pointer to Rx buffer, Data length
 * Return = void
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t DataLenght);



/* IRQ configuration ISR handling*/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQState);              //Enabling the interrupt etc...
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);           // Set Priority
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);                                //What to do if interrupt happens

/*
 * Other SPI APIs
 * SPIx = SPI1, SPI2,...,etc
 * SPIStatus = Enable / Disable
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t Status);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t Status);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t Status);

#endif /* INC_STM32F407XX_SPI_H_ */
