/*
 * stm32f407xx_GPIO.h
 *
 *  Created on: Mar 24, 2020
 *      Author: Hamzy
 */

#ifndef INC_STM32F407XX_GPIO_H_
#define INC_STM32F407XX_GPIO_H_

#include <stdint.h>
#include "stm32f407xx.h"



/** Configuration Structure for a GPIO pin **/
typedef struct
{
	uint8_t GPIO_PinNumber;         /*!< Possible values from @GPIO_PIN_NO >*/
	uint8_t GPIO_PinMode;           /*!< Possible values from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed;          /*!< Possible values from @GPIO_PIN_SPEED >*/
	uint8_t GPIO_PinPuPdControl;    /*!< Possible values from @GPIO_PIN_PUPD >*/
	uint8_t GPIO_PinOPType;         /*!< Possible values from @GPIO_OP_TYPE >*/
	uint8_t GPIO_PinAltFunMode;     /*!< Possible values from @GPIO_AF >*/
}GPIO_PinConfig_t;


/** Handle Structure for a GPIO pin **/
typedef struct
{
	GPIO_RegDef_t *pGPIOx;                // this hold the base address of the GPIO port the pin belongs to
	GPIO_PinConfig_t GPIO_PinConfig;      // this hold GPIO pin configuration settings
}GPIO_Handle_t;



/* @GPIO_PIN_NO
 * GPIO pin possible number
 */
#define GPIO_PIN_0           0
#define GPIO_PIN_1           1
#define GPIO_PIN_2           2
#define GPIO_PIN_3           3
#define GPIO_PIN_4           4
#define GPIO_PIN_5           5
#define GPIO_PIN_6           6
#define GPIO_PIN_7           7
#define GPIO_PIN_8           8
#define GPIO_PIN_9           9
#define GPIO_PIN_10          10
#define GPIO_PIN_11          11
#define GPIO_PIN_12          12
#define GPIO_PIN_13          13
#define GPIO_PIN_14          14
#define GPIO_PIN_15          15


/* @GPIO_PIN_MODES
 * GPIO pin possible mode
 */
#define GPIO_MODE_IN          0
#define GPIO_MODE_OUT         1
#define GPIO_MODE_ALTFN       2
#define GPIO_MODE_ANALOG      3
#define GPIO_MODE_INT_FT      4  // Interrupt Falling Edge Trigger
#define GPIO_MODE_INT_RT      5  // Interrupt Rising Edge Trigger
#define GPIO_MODE_INT_FRT     6  // Interrupt Falling & Rising Edge Trigger



/* @GPIO_OP_TYPE
 * GPIO pin possible output type
 */
#define GPIO_OP_TYPE_PP          0    // push pull  0/1
#define GPIO_OP_TYPE_OD          1    // open drain 0/noise




/* @GPIO_PIN_SPEED
 * GPIO pin possible output speed
 */



#define GPIO_SPEED_LOW           0
#define GPIO_SPEED_MEDUIM        1
#define GPIO_SPEED_FAST          2
#define GPIO_SPEED_HIGH          3




/* @GPIO_PIN_PUPD
 * GPIO pin PULL UP / PULL DOWN configuration macros
 */

#define GPIO_NO_PUPD            0
#define GPIO_PIN_PU             1
#define GPIO_PIN_PD             2



/* @GPIO_AF
 * GPIO possible Alternate Functions
 */
#define GPIO_AF0                0
#define GPIO_AF1                1
#define GPIO_AF2                2
#define GPIO_AF3                3
#define GPIO_AF4                4
#define GPIO_AF5                5
#define GPIO_AF6                6
#define GPIO_AF7                7
#define GPIO_AF8                8
#define GPIO_AF9                9
#define GPIO_AF10               10
#define GPIO_AF11               11
#define GPIO_AF12               12
#define GPIO_AF13               13
#define GPIO_AF14               14
#define GPIO_AF15               15


/*******************************************************************************
 *                        APIs supported by this driver
 *       for more information about the APIs check the functions definitions
 *
 *******************************************************************************/
/* Peripheral clock setup  */
/*Arguments == base address of GPIO port, Value of clock (SET/RESET)
 * Return == void                                                   */
void GPIO_PClkControl(GPIO_RegDef_t *pGPIOx, uint8_t clkValue);



/* GPIO Init/Dinit Functions*/
/**Arguments == pointer to Handle structure
 * Return = void**/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);  // there is a single register to shut down each peripheral clock RCC_AHB1RSTR



/* GPIO Data Read/Write Functions*/

/**Arguments = Base address of GPIOx, Pin number
 * Return = Pin value**/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/**Arguments = Base address of GPIOx
 * Return = Port value**/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

/**Arguments = Base address of GPIOx, Pin number, Pin Value
 * Return = void**/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t PinVlaue);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t PortValue);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);




/* IRQ configuration ISR handling*/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQState);             //Enabling the interrupt etc...
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);           // Set Priority
void GPIO_IRQHandling(uint8_t PinNumber);                                      //What to do if interrupt happens



#endif /* INC_STM32F407XX_GPIO_H_ */
