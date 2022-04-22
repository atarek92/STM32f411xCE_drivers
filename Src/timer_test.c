/*
 * timer_test.c
 *
 *  Created on: Apr 17, 2022
 *      Author: atarek
 */

#include <STM32f407xx_TIMER.h>
#include "STM32f407xx_GPIO.h"

#define LED_PORT 				GPIOA
#define LED_PIN 				GPIO_PIN_NO_5

#define BUTTON_PORT 			GPIOC
#define BUTTON_PIN 				GPIO_PIN_NO_13

int main (void)
{
	GPIO_Handle_t GpioLed, GPIOBtn;

	/*  LED GPIO configuration */
	GpioLed.pGPIOx = LED_PORT;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = LED_PIN;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GpioLed);

	/* Button GPIO configuration */
	GPIOBtn.pGPIOx = BUTTON_PORT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = BUTTON_PIN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GPIOBtn);

	/* Timer configuration */
	TIM_ADVANCED_Handle_t timer_handle;
	timer_handle.pTIMx=TIM1;
	timer_handle.TIM_Config.TIM_Prescaler = 1600;
	timer_handle.TIM_Config.TIM_AutoRelaod = 10000;
    TIM_ADVANCED_init(&timer_handle);

	while (1)
	{
		GPIO_ToggleOutputPin(LED_PORT,LED_PIN);
		while (!Check_TIM_ADVANCED_Interrupt_flag(&timer_handle));
//		systickDelayMs(1000);

	}
	return 0;
}
