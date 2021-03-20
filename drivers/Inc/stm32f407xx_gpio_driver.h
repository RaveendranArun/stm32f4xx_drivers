/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Mar 14, 2021
 *      Author: Raveendran Arun
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_


#include "stm32f407xx.h"

/*
 * Configuration structure for a GPIO pin
 */
typedef struct _GPIO_PinConfig_t
{
	uint8_t GPIO_PinNumber;              /*!< Possible values from @GPIO_PIN_NUMBERS */
	uint8_t GPIO_PinMode;                /*!< Possible values from @GPIO_PIN_MODES */
	uint8_t GPIO_PinSpeed;               /*!< Possible values from @GPIO_PIN_SPEEDS */
	uint8_t GPIO_PinPuPdControl;         /*!< Possible values from @GPIO_PIN_PUPD */
	uint8_t GPIO_PinOPType;              /*!< Possible values from @GPIO_PIN_OUT_TYPES */
	uint8_t GPIO_PinAltFunction;         /*!< GPIO pin alternate function */
}GPIO_PinConfig_t;

/*
 * Handle structure for a GPIO pin
 */
typedef struct
{
	GPIO_RegDef_t*     pGPIOx;              /*!< Holds the base address of the GPIO port to which the pin belongs */
	GPIO_PinConfig_t   GPIO_PinConfig;      /*!< Holds the GPIO pin configuration settings */
}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0           0
#define GPIO_PIN_NO_1           1
#define GPIO_PIN_NO_2           2
#define GPIO_PIN_NO_3           3
#define GPIO_PIN_NO_4           4
#define GPIO_PIN_NO_5           5
#define GPIO_PIN_NO_6           6
#define GPIO_PIN_NO_7           7
#define GPIO_PIN_NO_8           8
#define GPIO_PIN_NO_9           9
#define GPIO_PIN_NO_10          10
#define GPIO_PIN_NO_11          11
#define GPIO_PIN_NO_12          12
#define GPIO_PIN_NO_13          13
#define GPIO_PIN_NO_14          14
#define GPIO_PIN_NO_15          15

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN            0           /*!< Input (reset state) */
#define GPIO_MODE_OUT           1           /*!< General purpose output mode */
#define GPIO_MODE_ALTFN         2           /*!< Alternate function mode */
#define GPIO_MODE_ANALOG        3           /*!< Analog mode */
#define GPIO_MODE_IT_FT         4           /*!< Interrupt falling edge mode */
#define GPIO_MODE_IT_RT         5           /*!< Interrupt rising edge mode */
#define GPIO_MODE_IT_RFT        6           /*!< Interrupt rising edge, falling edge mode */

/*
 * @GPIO_PIN_OUT_TYPES
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP         0           /*!< Output push-pull (reset state) */
#define GPIO_OP_TYPE_OD         1           /*!< Output open-drain */

/*
 * @GPIO_PIN_SPEEDS
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW          0           /*!< Low speed */
#define GPIO_SPEED_MEDIUM       1           /*!< Medium speed */
#define GPIO_SPEED_FAST         2           /*!< High speed */
#define GPIO_SPEED_HIGH         3           /*!< Very high speed */

/*
 * @GPIO_PIN_PUPD
 * GPIO pin pull up and pull down configurations
 */
#define GPIO_NO_PUPD            0
#define GPIO_PIN_PU             1
#define GPIO_PIN_PD             2

/*********************************************************************************************
 *                                APIS supported by this driver
 *        For more information about the APIs please check the function definitions
 *********************************************************************************************/

/*
 * Peripheral clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t* pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t uPinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t uPinNumber, uint16_t uValue);
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t uValue);
void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t uPinNumber);

/*
 * IRQ configuration and IRQ handling
 */
void GPIO_IRQInterruptConfig(uint8_t uIRQNumber, uint8_t uEnOrDi);
void GPIO_IRQPriorityConfig(uint8_t uIRQNumber, uint32_t uIRQPriority);
void GPIO_IRQHandling(uint8_t uPinNumber);


#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
