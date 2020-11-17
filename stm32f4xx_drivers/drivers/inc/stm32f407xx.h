/*
 * stm32f404xx.h
 *
 *  Created on: Mar 14, 2020
 *      Author: Hamzy
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <stddef.h>

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

/************************************** I2C peripheral ******************************/
typedef struct
{
	volatile uint32_t CR1;             // 0x00 I2C control register-1
	volatile uint32_t CR2;             // 0x04 I2C control register-2
	volatile uint32_t OAR1;            // 0x08 I2C OAR2 register
	volatile uint32_t OAR2;            // 0x0C I2C OAR2 register
	volatile uint32_t DR;              // 0x10 I2C data register
	volatile uint32_t SR1;             // 0x14 I2C status register-1
	volatile uint32_t SR2;             // 0x18 I2C status register-2
	volatile uint32_t CCR;             // 0x1C I2C clock configuration register
	volatile uint32_t TRISE;           // 0x20 I2C TRISE register
	volatile uint32_t FLTR;            // 0x20 I2C FLTR register
}I2C_RegDef_t;


/************************************** USART peripheral ******************************/
typedef struct
{
	volatile uint32_t SR;                // 0x00 USART status register
	volatile uint32_t DR;                // 0x04 USART data register
	volatile uint32_t BRR;               // 0x08 USART baud rate register
	volatile uint32_t CR1;               // 0x0C USART control register-1
	volatile uint32_t CR2;               // 0x10 USART control register-2
	volatile uint32_t CR3;               // 0x14 USART control register-3
	volatile uint32_t GTPR;              // 0x18 USART guard time & prescaler register
}USART_RegDef_t;

/***************************************************************************************************************
 * *********** peripheral definition Macros == peripheral base address type casted (xxx_RegDef_t *) ************
 * ************************************************************************************************************/
// @GPIOx
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

// @SPIx
#define SPI1                            ( (SPI_RegDef_t*) SPI1_BASEADDR )
#define SPI2                            ( (SPI_RegDef_t*) SPI2_BASEADDR )
#define SPI3                            ( (SPI_RegDef_t*) SPI3_BASEADDR )
#define SPI4                            ( (SPI_RegDef_t*) SPI4_BASEADDR )


// @I2Cx
#define I2C1                            ( (I2C_RegDef_t*) I2C1_BASEADDR )
#define I2C2                            ( (I2C_RegDef_t*) I2C2_BASEADDR )
#define I2C3                            ( (I2C_RegDef_t*) I2C3_BASEADDR )


// @USARTx
#define USART1                          ( (USART_RegDef_t*) USART1_BASEADDR )
#define USART2                          ( (USART_RegDef_t*) USART2_BASEADDR )
#define USART3                          ( (USART_RegDef_t*) USART3_BASEADDR )
#define UART4                           ( (USART_RegDef_t*) UART4_BASEADDR )
#define UART5                           ( (USART_RegDef_t*) UART5_BASEADDR )
#define USART6                          ( (USART_RegDef_t*) USART6_BASEADDR )



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
#define UART4_PCLK_EN()                ( RCC->APB1ENR |= (1<<19) )
#define UART5_PCLK_EN()                ( RCC->APB1ENR |= (1<<20) )
#define USART6_PCLK_EN()               ( RCC->APB1ENR |= (1<<5) )


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
#define UART4_PCLK_DI()                ( RCC->APB1ENR &= ~(1<<19) )



/* Clock Disable Macros for SYSCFG Peripherals*/
#define SYSCFG_PCLK_DI()               ( RCC->APB2ENR &= ~(1<<14) )




/* Peripheral RESET Macros for GPIOx Peripherals*/
#define GPIOA_REG_RESET()              do { ( RCC->AHB1RSTR |= (1<<0) ); ( RCC->AHB1RSTR &= ~(1<<0) ); } while(0)
#define GPIOB_REG_RESET()              do { ( RCC->AHB1RSTR |= (1<<1) ); ( RCC->AHB1RSTR &= ~(1<<1) ); } while(0)
#define GPIOC_REG_RESET()              do { ( RCC->AHB1RSTR |= (1<<2) ); ( RCC->AHB1RSTR &= ~(1<<2) ); } while(0)
#define GPIOD_REG_RESET()              do { ( RCC->AHB1RSTR |= (1<<3) ); ( RCC->AHB1RSTR &= ~(1<<3) ); } while(0)
#define GPIOE_REG_RESET()              do { ( RCC->AHB1RSTR |= (1<<4) ); ( RCC->AHB1RSTR &= ~(1<<4) ); } while(0)



/* Peripheral RESET Macros for SPIx Peripherals */
#define SPI1_REG_RESET()              do { ( RCC->APB2RSTR |= (1<<12) ); ( RCC->APB2RSTR &= ~(1<<12) ); } while(0)
#define SPI2_REG_RESET()              do { ( RCC->APB1RSTR |= (1<<14) ); ( RCC->APB1RSTR &= ~(1<<14) ); } while(0)
#define SPI3_REG_RESET()              do { ( RCC->APB1RSTR |= (1<<15) ); ( RCC->APB1RSTR &= ~(1<<15) ); } while(0)
#define SPI4_REG_RESET()              do { ( RCC->APB2RSTR |= (1<<13) ); ( RCC->APB2RSTR &= ~(1<<13) ); } while(0)




/* Peripheral RESET Macros for I2Cx Peripherals */
#define I2C1_REG_RESET()              do { ( RCC->APB1RSTR |= (1<<21) ); ( RCC->APB1RSTR &= ~(1<<21) ); } while(0)
#define I2C2_REG_RESET()              do { ( RCC->APB1RSTR |= (1<<22) ); ( RCC->APB1RSTR &= ~(1<<22) ); } while(0)
#define I2C3_REG_RESET()              do { ( RCC->APB1RSTR |= (1<<23) ); ( RCC->APB1RSTR &= ~(1<<23) ); } while(0)



/* Peripheral RESET Macros for USARTx Peripherals */
#define USART1_REG_RESET()            do { ( RCC->APB2RSTR |= (1<<4) ); ( RCC->APB2RSTR &= ~(1<<4) ); } while(0)
#define USART2_REG_RESET()            do { ( RCC->APB1RSTR |= (1<<17) ); ( RCC->APB1RSTR &= ~(1<<17) ); } while(0)
#define USART3_REG_RESET()            do { ( RCC->APB1RSTR |= (1<<18) ); ( RCC->APB1RSTR &= ~(1<<18) ); } while(0)
#define UART4_REG_RESET()             do { ( RCC->APB1RSTR |= (1<<19) ); ( RCC->APB1RSTR &= ~(1<<19) ); } while(0)
#define UART5_REG_RESET()             do { ( RCC->APB1RSTR |= (1<<20) ); ( RCC->APB1RSTR &= ~(1<<20) ); } while(0)
#define USART6_REG_RESET()            do { ( RCC->APB2RSTR |= (1<<5) ); ( RCC->APB2RSTR &= ~(1<<5) ); } while(0)


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


#define IRQ_NO_SPI1                      35
#define IRQ_NO_SPI2                      36
#define IRQ_NO_SPI3                      51


#define IRQ_NO_I2C1_EV                   31  // I2C1 event interrupt
#define IRQ_NO_I2C1_ER                   32  // I2C1 error interrupt

#define IRQ_NO_I2C2_EV                   33  // I2C2 event interrupt
#define IRQ_NO_I2C2_ER                   34  // I2C2 error interrupt

#define IRQ_NO_I2C3_EV                   72  // I2C3 event interrupt
#define IRQ_NO_I2C3_ER                   73  // I2C3 error interrupt

#define IRQ_NO_USART1                    37  // USART1 global interrupt
#define IRQ_NO_USART2                    38  // USART2 global interrupt
#define IRQ_NO_USART3                    39  // USART3 global interrupt
#define IRQ_NO_UART4                     52  // USART4 global interrupt
#define IRQ_NO_UART5                     53  // USART5 global interrupt
#define IRQ_NO_USART6                    71  // USART6 global interrupt


/*
 *  IRQ Possible Priorities levels
 *  --> @IRQPriority */
#define NVIC_IRQ_PRI0                    0
#define NVIC_IRQ_PRI1                    1
#define NVIC_IRQ_PRI2                    2
#define NVIC_IRQ_PRI15                   15
#define NVIC_IRQ_PRI42                   42
#define NVIC_IRQ_PRI58                   58

#define NVIC_IRQ_PRI44                   44  // USART1 interrupt priority
#define NVIC_IRQ_PRI45                   45  // USART2 interrupt priority
#define NVIC_IRQ_PRI46                   46  // USART2 interrupt priority



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




/********************************************************
 * Bit definition macros for I2C peripheral
********************************************************/

// CR1 register bits
#define I2C_CR1_PE             0  // Peripheral Enable
#define I2C_CR1_SMBUS          1  // SMBus mode
#define I2C_CR1_SMBTYPE        3  // SMBus type
#define I2C_CR1_ENARP          4  // ARP Enable
#define I2C_CR1_ENPEC          5  // PEC Enable
#define I2C_CR1_ENGC           6  // General Call Enable
#define I2C_CR1_NOSTRETCH      7  // Clock Stretch Disable (Slave Mode)
#define I2C_CR1_START          8  // Start Generation
#define I2C_CR1_STOP           9  // Stop Generation
#define I2C_CR1_ACK            10 // Acknowledge Enable
#define I2C_CR1_POS            11 // Acknowledge/PEC Position (for data reception)
#define I2C_CR1_PEC            12 // Packet error checking
#define I2C_CR1_ALERT          13 // SMBus alert
#define I2C_CR1_SWRST          15 // Software reset


// CR2 register bits
#define I2C_CR2_FREQ           0  // Peripheral clock frequency
#define I2C_CR2_ITERREN        8  // Error interrupt enable
#define I2C_CR2_ITEVTEN        9  // Event interrupt enable
#define I2C_CR2_ITBUFEN        10 // Buffer interrupt Enable
#define I2C_CR2_DMAEN          11 // DMA request enable
#define I2C_CR2_LAST           12 // DMA last transfer


// OAR1 Own address register bits
#define I2C_OAR1_ADD0          0  // Peripheral clock frequency
#define I2C_OAR1_ADD71         1  // Interface address
#define I2C_OAR1_ADD98         8  // Interface address
#define I2C_OAR1_ADDMODE      15 // Addressing mode (Slave mode)


// OAR2 Own address register bits
#define I2C_OAR2_ENDUAL        0  // Dual addressing mode
#define I2C_OAR2_ADD2          1  // Interface address of dual addressing


// Status register SR1
#define I2C_SR1_SB              0  // Start bit
#define I2C_SR1_ADDR            1  // Address sent (master mode) / matched (salve mode)
#define I2C_SR1_BTF             2  // Byte transfer finished
#define I2C_SR1_ADD10           3  // 10-bit header sent (master mode)
#define I2C_SR1_STOPF           4  // Stop detection (salve mode)
#define I2C_SR1_RXNE            6  // Data register empty Reception done
#define I2C_SR1_TXE             7  // Data register empty Transmission done
#define I2C_SR1_BERR            8  // Bus error
#define I2C_SR1_ARLO            9  // Arbitration lost (master mode)
#define I2C_SR1_AF              10 // Acknowledge failure
#define I2C_SR1_OVR             11 // Overrun / Underrun
#define I2C_SR1_PECERR          12 // PEC error in reception
#define I2C_SR1_TIMEOUT         14 // Timeout/Tlow error
#define I2C_SR1_SMBALERT        15 // SMBus alert


// Status register SR2
#define I2C_SR2_MSL             0  // Master/Slave mode
#define I2C_SR2_BUSY            1  // Bus busy
#define I2C_SR2_TRA             2  // Transmitter/receiver
#define I2C_SR2_GENCALL         4  // General call address (slave mode)
#define I2C_SR2_SMBDEFAULT      5  // SMBus device default address (slave mode)
#define I2C_SR2_SMBHOST         6  // SMBus host address (Slave mode)
#define I2C_SR2_DUALF           7  // Dual flag (slave mode)



// Clock control register CCR
#define I2C_CCR_CCR             0  // Clock control register in Fm/Sm mode (master mode)
#define I2C_CCR_DUTY            14 // Fm mode duty cycle
#define I2C_CCR_FS              15 // Master mode selection

/*******************************************************
 * Bit definition macros for USART peripheral
********************************************************/
// Status Register SR
#define USART_SR_PE              0 // Parity error
#define USART_SR_FE              1 // Framing error
#define USART_SR_NF              2 // Noise detected Flag
#define USART_SR_ORE             3 // Overrun error
#define USART_SR_IDLE            4 // Idle line detected
#define USART_SR_RXNE            5 // Read data register not empty
#define USART_SR_TC              6 // Transmission complete
#define USART_SR_TXE             7 // Transmit data register empty
#define USART_SR_LBD             8 // LIN break detection flag
#define USART_SR_CTS             9 // CTS flag

// Status Register CR1
#define USART_CR1_SBK              0  // Send break
#define USART_CR1_RWU              1  // Receiver wakeup
#define USART_CR1_RE               2  // Receiver enable
#define USART_CR1_TE               3  // Transmitter enable
#define USART_CR1_IDLEIE           4  // Idle interrupt enable
#define USART_CR1_RXNEIE           5  // RXNE interrupt enable
#define USART_CR1_TCIE             6  // Transmission complete interrupt enable
#define USART_CR1_TXEIE            7  // TXE interrupt enable
#define USART_CR1_PEIE             8  // PE interrupt enable
#define USART_CR1_PS               9  // Parity selection
#define USART_CR1_PCE              10 // Parity control enable
#define USART_CR1_WAKE             11 // Wakeup method
#define USART_CR1_M                12 // Word length
#define USART_CR1_UE               13 // USART enable
#define USART_CR1_OVER8            15 // Oversampling mode


// Status Register CR2
#define USART_CR2_ADD              0  // Address of USART node
#define USART_CR2_LBDL             5  // LIN break detection length
#define USART_CR2_LBDIE            6  // LIN break detection interrupt enable
#define USART_CR2_LBCL             8  // Last bit clock bit
#define USART_CR2_CPHA             9  // Clock phase
#define USART_CR2_CPOL             10 // Clock polarity
#define USART_CR2_CLKEN            11  // Clock enable
#define USART_CR2_STOP             12  // STOP bits
#define USART_CR2_LINEN            14  // LIN mode enable



// Status Register CR3
// Status Register CR2
#define USART_CR3_EIE              0  // Error interrupt enable
#define USART_CR3_IREN             1  // IrDA mode enable
#define USART_CR3_IRLP             2  // IrDA low-power
#define USART_CR3_HDSEL            3  // Half-duplex selection
#define USART_CR3_NACK             4  // smart card NACK enable
#define USART_CR3_SCEN             5  // smart card mode enable
#define USART_CR3_DMAR             6  // DMA enable receiver
#define USART_CR3_DMAT             7  // DMA enable transmitter
#define USART_CR3_RTSE             8  // RTS enable
#define USART_CR3_CTSE             9  // CTS enable
#define USART_CR3_CTSIE            10  // CTS interrupt enable
#define USART_CR3_ONEBIT           11  // One sample bit method enable





/********************************************************
 * Bit definition macros for RCC peripheral
********************************************************/
// Clock configuration register CFGR
#define RCC_CFGR_SW             0  // System clock switch (By SW)
#define RCC_CFGR_SWS            2  // System clock switch status (By HW) @RCC_CFGR_SWS "RCC.h"
#define RCC_CFGR_HPRE           4  // AHB prescaler @RCC_CFGR_HPRE "RCC.h"
#define RCC_CFGR_PPRE1          10 // APB1 low speed prescaler @RCC_CFGR_PPRE "RCC.h"
#define RCC_CFGR_PPRE2          13 // APB1 high speed prescaler



/* will be included directly with the stm32f407 */
#include "stm32f407xx_GPIO.h"
#include "stm32f407xx_SPI.h"
#include "stm32f407xx_I2C.h"
#include "stm32f407xx_RCC.h"
#include "stm32f407xx_USART.h"

#endif /* INC_STM32F407XX_H_ */
