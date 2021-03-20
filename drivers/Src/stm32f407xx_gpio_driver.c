/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Mar 14, 2021
 *  Author: Raveendran Arun
 */

#include "stm32f407xx_gpio_driver.h"


/*
 * Peripheral clock setup
 */

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - Base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
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
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
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
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */

void GPIO_Init(GPIO_Handle_t* pGPIOHandle)
{
	uint32_t val = 0;

	/* 1. Configure the mode of the pin */
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		val = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		/* Clearing */
		pGPIOHandle->pGPIOx->MODER &= ~( 0x03 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		/* Setting */
		pGPIOHandle->pGPIOx->MODER |= val;
		val = 0;
	}
	else /* Interrupt Mode */
	{
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			/* 1. Configure the FTSR */
			/* Clearing */
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			/* Setting */
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			/* Clear the corresponding RTSR */
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			/* 1. Configure the RTSR */
			/* Clearing */
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			/* Setting */
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			/* Clear the corresponding FTSR */
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{

			/* Clearing */
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			/* 1. Configure FTSR and RTSR */
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		/* 2. Configure the port selection in SYSCFG_EXTICR */
		uint8_t uExtiIndex = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t uExtiPinNumber = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t uPortCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		/*Enable the clock for SYSCFG */
		SYSCFG_PCLK_EN();
		/*Clearing */
		SYSCFG->EXTICR[uExtiIndex] &= ~( 0xFF << (4 * uExtiPinNumber));
		/*Setting */
		SYSCFG->EXTICR[uExtiIndex] |= ( uPortCode << (4 * uExtiPinNumber));

		/* 3. Enable EXTI interrupt delivery using IMR */
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	/* 2. Configure the speed of the pin */
	val = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	/* Clearing */
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x03 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	/* Setting */
	pGPIOHandle->pGPIOx->OSPEEDR |= val;
	val = 0;

	/* 3. Configure the pull up/down settings */
	val = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	/* Clearing */
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x03 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	/* Setting */
	pGPIOHandle->pGPIOx->PUPDR |= val;
	val = 0;

	/* 4. Configure the output type */
	val = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	/* Clearing */
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x01 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	/* Setting */
	pGPIOHandle->pGPIOx->OTYPER |= val;
	val = 0;

	/* 5. Configure the alt functionality */
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t uAfrIndex = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		uint8_t uAfrPinNumber = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		val = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunction << (4 * uAfrPinNumber);
		/* Clearing */
		pGPIOHandle->pGPIOx->AFR[uAfrIndex] &= ~(0x0F << (4 * uAfrPinNumber));
		/* Setting */
		pGPIOHandle->pGPIOx->AFR[uAfrIndex] |= val;
		val = 0;
	}
}

void GPIO_DeInit(GPIO_RegDef_t* pGPIOx)
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
	else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if (pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
}

/*
 * Data read and write
 */

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -   0 or 1
 *
 * @Note              -
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t uPinNumber)
{
	uint8_t uData = 0;

	uData = (uint8_t)( (pGPIOx->IDR >> uPinNumber) & 0x01 );

	return  uData;
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */


uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx)
{
	uint16_t uData = 0;

	uData = (uint16_t)(pGPIOx->IDR);

	return uData;
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t uPinNumber, uint16_t uValue)
{
	if (uValue == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << uPinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << uPinNumber);
	}
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t uValue)
{
	pGPIOx->ODR = uValue;
}

/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t uPinNumber)
{
	pGPIOx->ODR ^= (1 << uPinNumber);
}

/*
 * IRQ configuration and IRQ handling
 */

void GPIO_IRQInterruptConfig(uint8_t uIRQNumber, uint8_t uEnorDi)
{
	if (uEnorDi == ENABLE)
	{
		if (uIRQNumber <= 31)
		{
			/* Programm ISER0 */
			*NVIC_ISER0 |= (1 << uIRQNumber);
		}
		else if ( (uIRQNumber > 31) && (uIRQNumber < 64) )
		{
			/* Programm ISER1 */
			*NVIC_ISER1 |= (1 << (uIRQNumber % 32));
		}
		else if ( (uIRQNumber >= 64) && (uIRQNumber < 96) )
		{
			/* Programm ISER2 */
			*NVIC_ISER2 |= (1 << (uIRQNumber % 64));
		}
	}
	else
	{
		if (uIRQNumber <= 31)
		{
			/* Programm ICER0 */
			*NVIC_ICER0 |= (1 << uIRQNumber);
		}
		else if ( (uIRQNumber > 31) && (uIRQNumber < 64) )
		{
			/* Programm ICER1 */
			*NVIC_ICER1 |= (1 << (uIRQNumber % 32));
		}
		else if ( (uIRQNumber >= 64) && (uIRQNumber < 96) )
		{
			/* Programm ICER2 */
			*NVIC_ICER2 |= (1 << (uIRQNumber % 64));
		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t uIRQNumber, uint32_t uIRQPriority)
{
	/* IPRx register */
	uint8_t uIPRx = uIRQNumber / 4;
	/* Section of IPRx register*/
	uint8_t uIPRSection = uIRQNumber % 4;
	/* Shift amount: highest nibble only implemented out of 8 bits */
	uint8_t uShiftAmount = (8 * uIPRSection) + (8 - NUM_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASEADDR +  uIPRx) |= (uIRQPriority << uShiftAmount);
}

void GPIO_IRQHandling(uint8_t uPinNumber)
{
	/* Clear the EXTI PR register corresponding to the pin number */
	if ( (EXTI->PR) & (1 << uPinNumber) )
	{
		/* Clear the register by writing 1 to the corresponding pin number */
		EXTI->PR |= (1 << uPinNumber);
	}
}

