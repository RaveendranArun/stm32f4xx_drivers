/*
 * 002led_botton.c
 *
 *  Created on: 17-Mar-2021
 *      Author: Raveendran Arun
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

#define BTN_PRESSED  ENABLE

/* Some delay */
void delay(void)
{
	for (uint32_t i = 0; i < 500000 / 2; ++i);
}

int main()
{
	GPIO_Handle_t GpioLedRed, GpioBtn;

	/* Populate the GPIO handle for LED connected to PD14 */
	GpioLedRed.pGPIOx = GPIOD;
	GpioLedRed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GpioLedRed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLedRed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLedRed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLedRed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	/* Populate the GPIO handle for button connected to PA0 */
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	/* Enable the peripheral clk for GPIOD (LED) */
	GPIO_PeriClockControl(GpioLedRed.pGPIOx, ENABLE);

	/* Initialise the GPIOD */
	GPIO_Init(&GpioLedRed);

	/* Enable the peripheral clk for GPIOA (botton) */
	GPIO_PeriClockControl(GpioBtn.pGPIOx, ENABLE);

	/* Initialise the GPIOA */
	GPIO_Init(&GpioBtn);

	while(1)
	{
		if (GPIO_ReadFromInputPin(GpioBtn.pGPIOx, GpioBtn.GPIO_PinConfig.GPIO_PinNumber) == BTN_PRESSED)
		{
			delay();
			//+GPIO_ToggleOutputPin(GpioLedRed.pGPIOx , GpioLedRed.GPIO_PinConfig.GPIO_PinNumber);
			GPIO_WriteToOutputPin(GpioLedRed.pGPIOx, GpioLedRed.GPIO_PinConfig.GPIO_PinNumber, ENABLE);
		}
		else
		{
			GPIO_WriteToOutputPin(GpioLedRed.pGPIOx, GpioLedRed.GPIO_PinConfig.GPIO_PinNumber, DISABLE);
		}
	}

	return 0;
}


