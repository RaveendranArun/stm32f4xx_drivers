/*
 * 003button_interrupt.c
 *
 *  Created on: 20-Mar-2021
 *  Author: Raveendran Arun
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include <string.h>

/* Some delay */
void delay(void)
{
	/* This will introduce ~200ms delay when system clock is 16MHz */
	for (uint32_t i = 0; i < 500000 / 2; ++i);
}

void EXTI0_IRQHandler(void)
{
	delay(); // For button debouncing
 	GPIO_IRQHandling(GPIO_PIN_NO_0); // Clear the pending event from EXTI line
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_14); // Toggle the LED
}

int main()
{
	GPIO_Handle_t GpioLedRed, GpioBtn;

	/* Resetting the GPIO handles */
	memset(&GpioLedRed, 0, sizeof(GpioLedRed));
	memset(&GpioBtn, 0, sizeof(GpioBtn));

	/* Populate the GPIO handle for LED connected to PD14 */
	GpioLedRed.pGPIOx = GPIOD;
	GpioLedRed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GpioLedRed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLedRed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLedRed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLedRed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	/* Enable the peripheral clk for GPIOD (LED) */
	GPIO_PeriClockControl(GpioLedRed.pGPIOx, ENABLE);

	/* Initialise the GPIOD */
	GPIO_Init(&GpioLedRed);

	/* Populate the GPIO handle for button connected to PA0 */
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	/* Enable the peripheral clk for GPIOA (Button) */
	GPIO_PeriClockControl(GpioBtn.pGPIOx, ENABLE);

	/* Initialise the GPIOD */
	GPIO_Init(&GpioBtn);

	/* IRQ priority */
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRIO15);

	/* IRQ configuaration */
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);

	while(1);

	return 0;
}
