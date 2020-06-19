/*
 * stm32f404xx.h
 *
 *  Created on: Mar 14, 2020
 *      Author: Hamzy
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

/************************************ START: PROCESSOR SPECIFIC DETAILS ********************************/
/*
 *ARM Cortex-Mx  Processor NVIC ISER registers addresses
 */

#define NVIC_ISER0                     *( (volatile uint32_t*) 0xE000E100)
#define NVIC_ISER1                     *( (volatile uint32_t*) 0xE000E104)
#define NVIC_ISER2                     *( (volatile uint32_t*) 0xE000E108)
#define NVIC_ISER3                     *( (volatile uint32_t*) 0xE000E10C)


/*
 *ARM Cortex-Mx  Processor NVIC ICER registers addresses
 */
#define NVIC_ICER0                     *( (volatile uint32_t*) 0xE000E180)
#define NVIC_ICER1                     *( (volatile uint32_t*) 0xE000E184)
#define NVIC_ICER2                     *( (volatile uint32_t*) 0xE000E188)
#define NVIC_ICER3                     *( (volatile uint32_t*) 0xE000E18C)


/*
 *ARM Cortex-Mx  Processor NVIC IPR registers addresses
 */
#define NVIC_IPR_BASEADDR              ( (volatile uint32_t*) 0xE000E400) // removed * because it will be added in the program


#define NO_OF_PRIORITY_BITS    4            // Interrupt priorities bits MCU specific @prioritybits



/**************** Flash & SRAM base addresses **********************/
#define FLASH_BASEADDR                 0x08000000U           /* from reference manual/memory map this is the main memory*/
#define SRAM1_BASEADDR                 0x20000000U //112 KB  /* from reference manual*/
#define SRAM2_BASEADDR                 0x2001C000U //16 KB   /* calculated (112*1024) + 0x2000 0000*/
#define ROM_BASEADDR                   0x1FFF0000U //30 KB   /* from reference manual/embedded memory*/
#define SRAM                           SRAM1_BASEADDR


/**************** AHBx & APBx buses peripheral base address *****************/
#define PERIPH_BASEADDR                    0x40000000U
#define APB1PERIPH_BASEADDR                PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR                0x40010000U
#define AHB1PERIPH_BASEADDR                0x40020000U
#define AHB2PERIPH_BASEADDR                0x50000000U


/**************** Base Addresses for every single peripheral of the MCU *****************/

/**************** Base Addresses for peripherals hanging on AHB1 bus ********************/
#define GPIOA_BASEADDR                 (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR                 (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR                 (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR                 (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR                 (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR                 (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR                 (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR                 (AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR                 (AHB1PERIPH_BASEADDR + 0x2000)
#define GPIOJ_BASEADDR                 (AHB1PERIPH_BASEADDR + 0x2400)
#define GPIOK_BASEADDR                 (AHB1PERIPH_BASEADDR + 0x2800)

#define RCC_BASEADDR                   (AHB1PERIPH_BASEADDR + 0x3800)


/**************** Base Addresses for peripherals hanging on APB1 bus ********************/
#define I2C1_BASEADDR                  (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR                  (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR                  (APB1PERIPH_BASEADDR + 0x5C00)


#define SPI2_BASEADDR                  (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR                  (APB1PERIPH_BASEADDR + 0x3C00)


#define USART2_BASEADDR                (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR                (APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR                 (APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR                 (APB1PERIPH_BASEADDR + 0x5000)

/**************** Base Addresses for peripherals hanging on APB2 bus ********************/
#define USART1_BASEADDR                (APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR                (APB2PERIPH_BASEADDR + 0x1400)

#define SPI1_BASEADDR                  (APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR                  (APB2PERIPH_BASEADDR + 0x3400)


#define SYSCFG_BASEADDR                (APB2PERIPH_BASEADDR + 0x3800)


#define EXTI_BASEADDR                  (APB2PERIPH_BASEADDR + 0x3C00)





/************************************************************************************
 * ***************** peripheral register definition structures **********************
 * **********************************************************************************/


/***************************** GPIO peripheral ***************************/
typedef struct
{
	volatile uint32_t MODER;           // 0x00 GPIO port mode register
	volatile uint32_t OTYPER;          // 0x04 GPIO port output type register
	volatile uint32_t OSPEEDR;         // 0x08 GPIO port output speed register
	volatile uint32_t PUPDR;           // 0x0C GPIO port pull-up/pull-down register
	volatile uint32_t IDR;             // 0x10 GPIO port input data register
	volatile uint32_t ODR;             // 0x14 GPIO port output data register
	volatile uint32_t BSRR;            // 0x18 GPIO port bit set/reset register
	volatile uint32_t LCKR;            // 0x1C GPIO port configuration lock register
	volatile uint32_t AFR[2];          // 0x20 array of two instead of two register AFR[0] = AFRL & AFR[1] = AFRH
}GPIO_RegDef_t;



/********************** RCC peripheral ********************/
typedef struct
{
	volatile uint32_t CR;                // 0x00 RCC control register register
	volatile uint32_t PLLCFGR;           // 0x04 RCC PLL configuration register
	volatile uint32_t CFGR;              // 0x08 RCC clock configuration register
	volatile uint32_t CIR;               // 0x0C RCC clock interrupt register
	volatile uint32_t AHB1RSTR;          // 0x10 RCC AHB1 peripheral reset register
	volatile uint32_t AHB2RSTR;          // 0x14 RCC AHB2 peripheral reset register
	volatile uint32_t AHB3RSTR;          // 0x18 RCC AHB3 peripheral reset register
    uint32_t RESERVED0;                  // 0x1C
	volatile uint32_t APB1RSTR;          // 0x20 RCC APB1 peripheral reset register
	volatile uint32_t APB2RSTR;          // 0x24 RCC APB2 peripheral reset register
    uint32_t RESERVED1[2];               // 0x28
    volatile uint32_t AHB1ENR;           // 0x30 RCC AHB1 peripheral clock register
    volatile uint32_t AHB2ENR;           // 0x34 RCC AHB2 peripheral clock register
    volatile uint32_t AHB3ENR;           // 0x38 RCC AHB3 peripheral clock register
    uint32_t RESERVED2;                  // 0x2C
    volatile uint32_t APB1ENR;           // 0x40 RCC APB1 peripheral clock register
    volatile uint32_t APB2ENR;           // 0x44 RCC APB2 peripheral clock register
    uint32_t RESERVED3[2];               // 0x48
    volatile uint32_t AHB1LPENR;         // 0x50 RCC AHB1 peripheral clock LOW POWER MODE register
    volatile uint32_t AHB2LPENR;         // 0x54 RCC AHB2 peripheral clock LOW POWER MODE register
    volatile uint32_t AHB3LPENR;         // 0x58 RCC AHB3 peripheral clock LOW POWER MODE register
    uint32_t RESERVED4;                  // 0x5C
    volatile uint32_t APB1LPENR;         // 0x60 RCC APB1 peripheral clock LOW POWER MODE register
    volatile uint32_t APB2LPENR;         // 0x64 RCC APB2 peripheral clock LOW POWER MODE register
    uint32_t RESERVED5[2];               // 0x68
    volatile uint32_t BDCR;              // 0x70 RCC backup domain control register
    volatile uint32_t CSR;               // 0x74 RCC clock control & status register
    uint32_t RESERVED6[2];               // 0x78
    volatile uint32_t SSCGR;             // 0x80 RCC speed spectrum clock generation register
    volatile uint32_t PLLI2SCFGR;        // 0x84 RCC PLLI2S configuration register
    volatile uint32_t PLLSAICFGR;        // 0x88 RCC PLL configuration register
    volatile uint32_t DCKCFGR;           // 0x8C RCC dedicated clock configuration register
}RCC_RegDef_t;



/****************************** EXTI peripheral *******************************/
typedef struct
{
	volatile uint32_t EXTI_IMR;           // 0x00 EXTI interrupt mask register
	volatile uint32_t EXTI_EMR;           // 0x04 EXTI event mask register
	volatile uint32_t EXTI_RTSR;          // 0x08 EXTI rising trigger selection register
	volatile uint32_t EXTI_FTSR;          // 0x0C EXTI falling trigger selection register
	volatile uint32_t EXTI_SWIER;         // 0x10 EXTI software interrupt event register
	volatile uint32_t EXTI_PR;            // 0x14 EXTI pending register
}EXTI_RegDef_t;



/******************************** SYSCFG peripheral ***************************/
typedef struct
{
	volatile uint32_t MEMRMP;          // 0x00 SYSCFG memory re-map register
	volatile uint32_t PMC;             // 0x04 SYSCFG peripheral mode configuration register (PHY Ethernet)
	volatile uint32_t EXTICR[4];       // 0x08 - 0x14 SYSCFG external interrupt configuration registers 1:4 from EXTI0 : EXTI15
	 uint32_t RESEVED[2];              // 0x0C- 0x10 Reserved
	volatile uint32_t CMPCR;           // 0x20 Compensation cell control register
}SYSCFG_RegDef_t;


/************************************** SPI peripheral ******************************/
typedef struct
{
	volatile uint32_t CR1;             // 0x00 SPI control register-1
	volatile uint32_t CR2;             // 0x04 SPI control register-2
	volatile uint32_t SR;              // 0x08 SPI status register
	volatile uint32_t DR;              // 0x0C SPI data register
	volatile uint32_t CRCPR;           // 0x10 SPI polynomial register
	volatile uint32_t RXCRCR;          // 0x14 SPI RX CRC register
	volatile uint32_t TXCRCR;          // 0x18 SPI TX CRC register
	volatile uint32_t I2SCFGR;         // 0x1C I2S configuration register
	volatile uint32_t I2SPR;           // 0x20 I2S prescaler register
}SPI_RegDef_t;


/***************************************************************************************************************
 * *********** peripheral definition Macros == peripheral base address type casted (xxx_RegDef_t *) ************
 * ************************************************************************************************************/
#define GPIOA                           ( (GPIO_RegDef_t*) GPIOA_BASEADDR )
#define GPIOB                           ( (GPIO_RegDef_t*) GPIOB_BASEADDR )
#define GPIOC                           ( (GPIO_RegDef_t*) GPIOC_BASEADDR )
#define GPIOD                           ( (GPIO_RegDef_t*) GPIOD_BASEADDR )
#define GPIOE                           ( (GPIO_RegDef_t*) GPIOE_BASEADDR )
#define GPIOF                           ( (GPIO_RegDef_t*) GPIOF_BASEADDR )
#define GPIOG                           ( (GPIO_RegDef_t*) GPIOG_BASEADDR )
#define GPIOH                           ( (GPIO_RegDef_t*) GPIOH_BASEADDR )
#define GPIOI                           ( (GPIO_RegDef_t*) GPIOI_BASEADDR )
#define GPIOJ                           ( (GPIO_RegDef_t*) GPIOJ_BASEADDR )
#define GPIOK                           ( (GPIO_RegDef_t*) GPIOK_BASEADDR )

#define RCC                             ( (RCC_RegDef_t*) RCC_BASEADDR )


#define EXTI                            ( (EXTI_RegDef_t*) EXTI_BASEADDR )

#define SYSCFG                          ( (SYSCFG_RegDef_t*) SYSCFG_BASEADDR )

#define SPI1                            ( (SPI_RegDef_t*) SPI1_BASEADDR )
#define SPI2                            ( (SPI_RegDef_t*) SPI2_BASEADDR )
#define SPI3                            ( (SPI_RegDef_t*) SPI3_BASEADDR )
#define SPI4                            ( (SPI_RegDef_t*) SPI4_BASEADDR )


/**************************** Clock ENABLE MACROS ************************************/
/* Clock Enable Macros for GPIOx Peripherals*/
#define GPIOA_PCLK_EN()                 ( RCC->AHB1ENR |= (1<<0) )
#define GPIOB_PCLK_EN()                 ( RCC->AHB1ENR |= (1<<1) )
#define GPIOC_PCLK_EN()                 ( RCC->AHB1ENR |= (1<<2) )
#define GPIOD_PCLK_EN()                 ( RCC->AHB1ENR |= (1<<3) )
#define GPIOE_PCLK_EN()                 ( RCC->AHB1ENR |= (1<<4) )
#define GPIOF_PCLK_EN()                 ( RCC->AHB1ENR |= (1<<5) )
#define GPIOG_PCLK_EN()                 ( RCC->AHB1ENR |= (1<<6) )
#define GPIOH_PCLK_EN()                 ( RCC->AHB1ENR |= (1<<7) )
#define GPIOI_PCLK_EN()                 ( RCC->AHB1ENR |= (1<<8) )


/* Clock Enable Macros for I2Cx Peripherals*/
#define I2C1_PCLK_EN()                 ( RCC->APB1ENR |= (1<<21) )
#define I2C2_PCLK_EN()                 ( RCC->APB1ENR |= (1<<22) )
#define I2C3_PCLK_EN()                 ( RCC->APB1ENR |= (1<<23) )


/* Clock Enable Macros for SPIx Peripherals*/
#define SPI1_PCLK_EN()                 ( RCC->APB2ENR |= (1<<12) )
#define SPI2_PCLK_EN()                 ( RCC->APB1ENR |= (1<<14) )
#define SPI3_PCLK_EN()                 ( RCC->APB1ENR |= (1<<15) )
#define SPI4_PCLK_EN()                 ( RCC->APB2ENR |= (1<<13) )


/* Clock Enable Macros for UARTx Peripherals*/
#define USART1_PCLK_EN()               ( RCC->APB2ENR |= (1<<4) )
#define USART2_PCLK_EN()               ( RCC->APB1ENR |= (1<<17) )
#define USART3_PCLK_EN()               ( RCC->APB1ENR |= (1<<18) )


/* Clock Enable Macros for SYSCFG Peripherals*/
#define SYSCFG_PCLK_EN()               ( RCC->APB2ENR |= (1<<14) )



/* Clock Disable Macros for GPIOx Peripherals*/
#define GPIOA_PCLK_DI()                 ( RCC->AHB1ENR &= ~(1<<0) )
#define GPIOB_PCLK_DI()                 ( RCC->AHB1ENR &= ~(1<<1) )
#define GPIOC_PCLK_DI()                 ( RCC->AHB1ENR &= ~(1<<2) )
#define GPIOD_PCLK_DI()                 ( RCC->AHB1ENR &= ~(1<<3) )
#define GPIOE_PCLK_DI()                 ( RCC->AHB1ENR &= ~(1<<4) )
#define GPIOF_PCLK_DI()                 ( RCC->AHB1ENR &= ~(1<<5) )
#define GPIOG_PCLK_DI()                 ( RCC->AHB1ENR &= ~(1<<6) )
#define GPIOH_PCLK_DI()                 ( RCC->AHB1ENR &= ~(1<<7) )
#define GPIOI_PCLK_DI()                 ( RCC->AHB1ENR &= ~(1<<8) )



/* Clock Disable Macros for I2Cx Peripherals*/
#define I2C1_PCLK_DI()                 ( RCC->APB1ENR &= ~(1<<21) )
#define I2C2_PCLK_DI()                 ( RCC->APB1ENR &= ~(1<<22) )
#define I2C3_PCLK_DI()                 ( RCC->APB1ENR &= ~(1<<23) )



/* Clock Disable Macros for SPIx Peripherals*/
#define SPI1_PCLK_DI()                 ( RCC->APB2ENR &= ~(1<<12) )
#define SPI2_PCLK_DI()                 ( RCC->APB1ENR &= ~(1<<14) )
#define SPI3_PCLK_DI()                 ( RCC->APB1ENR &= ~(1<<15) )
#define SPI4_PCLK_DI()                 ( RCC->APB2ENR &= ~(1<<13) )



/* Clock Disable Macros for UARTx Peripherals*/
#define USART1_PCLK_DI()               ( RCC->APB2ENR &= ~(1<<4) )
#define USART2_PCLK_DI()               ( RCC->APB1ENR &= ~(1<<17) )
#define USART3_PCLK_DI()               ( RCC->APB1ENR &= ~(1<<18) )



/* Clock Disable Macros for SYSCFG Peripherals*/
#define SYSCFG_PCLK_DI()               ( RCC->APB2ENR &= ~(1<<14) )




/* Peripheral RESET Macros for GPIOx Peripherals*/
#define GPIOA_REG_RESET()              do { ( RCC->AHB1RSTR |= (1<<0) ); ( RCC->AHB1RSTR &= ~(1<<0) ); } while(0)
#define GPIOB_REG_RESET()              do { ( RCC->AHB1RSTR |= (1<<1) ); ( RCC->AHB1RSTR &= ~(1<<1) ); } while(0)
#define GPIOC_REG_RESET()              do { ( RCC->AHB1RSTR |= (1<<2) ); ( RCC->AHB1RSTR &= ~(1<<2) ); } while(0)
#define GPIOD_REG_RESET()              do { ( RCC->AHB1RSTR |= (1<<3) ); ( RCC->AHB1RSTR &= ~(1<<3) ); } while(0)
#define GPIOE_REG_RESET()              do { ( RCC->AHB1RSTR |= (1<<4) ); ( RCC->AHB1RSTR &= ~(1<<4) ); } while(0)



/* Peripheral RESET Macros for SPIx Peripherals*/
#define SPI1_REG_RESET()              do { ( RCC->APB2RSTR |= (1<<12) ); ( RCC->APB2RSTR &= ~(1<<12) ); } while(0)
#define SPI2_REG_RESET()              do { ( RCC->APB1RSTR |= (1<<14) ); ( RCC->APB1RSTR &= ~(1<<14) ); } while(0)
#define SPI3_REG_RESET()              do { ( RCC->APB1RSTR |= (1<<15) ); ( RCC->APB1RSTR &= ~(1<<15) ); } while(0)
#define SPI4_REG_RESET()              do { ( RCC->APB2RSTR |= (1<<13) ); ( RCC->APB2RSTR &= ~(1<<13) ); } while(0)



/* This macro returns a code (0 to 7) for a given GPIO base address*/
#define GPIO_PORT_EXTI_CODE(port)     ( (port == GPIOA) ? 0 :\
		                                (port == GPIOB) ? 1 :\
		                                (port == GPIOC) ? 2 :\
		                                (port == GPIOD) ? 3 :\
		                                (port == GPIOE) ? 4 :\
		                                (port == GPIOF) ? 5 :\
		                                (port == GPIOG) ? 6 :\
		                                (port == GPIOH) ? 7 : 8 )


/*
 *  IRQ (Interrupt Request) number for STM32F407x MCU
 *  --> @IRQn */
#define IRQ_NO_EXTI0                     6
#define IRQ_NO_EXTI1                     7
#define IRQ_NO_EXTI2                     8
#define IRQ_NO_EXTI3                     9
#define IRQ_NO_EXTI4                     10
#define IRQ_NO_EXTI9_5                   23
#define IRQ_NO_EXTI15_10                 40




/*
 *  IRQ Possible Priorities levels
 *  --> @IRQPriority */
#define NVIC_IRQ_PRI0                    0
#define NVIC_IRQ_PRI1                    1
#define NVIC_IRQ_PRI2                    2
#define NVIC_IRQ_PRI15                   15




/* Generic Macros*/
#define ENABLE                 1
#define DISABLE                0
#define SET                    ENABLE
#define RESET                  DISABLE
#define GPIO_PIN_SET           SET
#define GPIO_PIN_RESET         RESET
#define FLAG_SET               SET
#define FLAG_RESET             RESET


/********************************************************
 * Bit definition macros for SPI peripheral
 ********************************************************/

// CR1 register bits
#define SPI_CR1_CPHA          0
#define SPI_CR1_CPOL          1
#define SPI_CR1_MSTR          2
#define SPI_CR1_BR            3  // 3, 4 & 5
#define SPI_CR1_SPE           6  // SPI Enable
#define SPI_CR1_LSB_FIRST     7
#define SPI_CR1_SSI           8
#define SPI_CR1_SSM           9
#define SPI_CR1_RXONLY        10
#define SPI_CR1_DFF           11
#define SPI_CR1_CRC_NEXT      12
#define SPI_CR1_CRC_EN        13
#define SPI_CR1_BIDIOE        14  // BI DI OUTPUT ENABLE
#define SPI_CR1_BIDIMODE      15


// CR2 register bits
#define SPI_CR2_RXDMAEN        0  // Rx buffer DMA enable
#define SPI_CR2_TXDMAEN        1  // Tx buffer DMA enable
#define SPI_CR2_SSOE           2  // SS out enable
#define SPI_CR2_FRF            4  // Frame format
#define SPI_CR2_ERRIE          5  // Error interrupt enable
#define SPI_CR2_RXNEIE         6  // Rx buffer not empty interrupt enable
#define SPI_CR2_TXEIE          7  // Tx buffer empty interrupt enable



// SR register bits
#define SPI_SR_RXNE            0  // Receive buffer not empty
#define SPI_SR_TXE             1  // Transmit buffer empty
#define SPI_SR_CHSIDE          2  // Channel Side
#define SPI_SR_UDR             3  // Underrun flag
#define SPI_SR_CRC_ERR         4  // CRC error fault
#define SPI_SR_MODF            5  // Mode fault flag
#define SPI_SR_OVR             6  // Overrun flag
#define SPI_SR_BSY             7  // Busy flag
#define SPI_SR_FRE             8  // Frame format error flag


/* will be included directly with the stm32f407 */
#include "stm32f407xx_GPIO.h"
#include "stm32f407xx_SPI.h"

#endif /* INC_STM32F407XX_H_ */
