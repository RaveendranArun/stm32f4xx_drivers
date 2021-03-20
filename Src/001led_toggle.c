/*
 * 001ledtoggle.c
 *
 *  Created on: 16-Mar-2021
 *      Author: Raveendran Arun
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

/* Some delay */
void delay(void)
{
	for (uint32_t i = 0; i < 40000; ++i);
}

int main()
{
	GPIO_Handle_t GpioLed, GpioLedRed, GpioLedOrange, GpioLedGreen, GpioLedBlue;

	/* Populate the GPIO handle */
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	/* Red */
	GpioLedRed = GpioLed;
	GpioLedRed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;

	/* Green */
	GpioLedGreen = GpioLed;
	GpioLedGreen.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;

	/* Orange */
	GpioLedOrange = GpioLed;
	GpioLedOrange.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;

	/* Blue */
	GpioLedBlue = GpioLed;
	GpioLedBlue.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;

	/* Enable the peripheral clk */
	GPIO_PeriClockControl(GpioLed.pGPIOx, ENABLE);

	/* Initialise the GPIO for Red*/
	GPIO_Init(&GpioLedRed);
	/* Initialise the GPIO for Green */
	GPIO_Init(&GpioLedGreen);
	/* Initialise the GPIO for Orange*/
	GPIO_Init(&GpioLedOrange);
	/* Initialise the GPIO for Blue */
	GPIO_Init(&GpioLedBlue);

	while(1)
	{
		GPIO_ToggleOutputPin(GpioLedGreen.pGPIOx, GpioLedGreen.GPIO_PinConfig.GPIO_PinNumber);
		delay();
		GPIO_ToggleOutputPin(GpioLedOrange.pGPIOx, GpioLedOrange.GPIO_PinConfig.GPIO_PinNumber);
		delay();
		GPIO_ToggleOutputPin(GpioLedRed.pGPIOx, GpioLedRed.GPIO_PinConfig.GPIO_PinNumber);
		delay();
		GPIO_ToggleOutputPin(GpioLedBlue.pGPIOx, GpioLedBlue.GPIO_PinConfig.GPIO_PinNumber);
		delay();
	}

	return 0;
}


