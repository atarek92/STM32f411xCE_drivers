/*
 * timer_interrupt_test.c
 *
 *  Created on: Apr 25, 2022
 *      Author: atarek
 */


#include <STM32f407xx_TIMER.h>
#include "STM32f407xx_GPIO.h"

#define LED_PORT 				GPIOA
#define LED_PIN 				GPIO_PIN_NO_5
TIM_ADVANCED_Handle_t timer_handle;

int main (void)
{
	GPIO_Handle_t GpioLed;

	/*  LED GPIO configuration */
	GpioLed.pGPIOx = LED_PORT;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = LED_PIN;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GpioLed);

	/* Timer configuration */

	timer_handle.pTIMx=TIM1;
	timer_handle.TIM_Config.TIM_Prescaler = 1600;
	timer_handle.TIM_Config.TIM_AutoRelaod = 10000;
	timer_handle.TIM_Config.TIM_InterruptEnable = ENABLE;
	timer_handle.TIM_Config.TIM_IRQNumber = IRQ_NO_TIM1_UP_TIM10;
    TIM_ADVANCED_init(&timer_handle);

	while (1)
	{

	}
	return 0;
}

void TIM1_UP_TIM10_IRQHandler()
{
	TIM_IRQHandling(&timer_handle);
	GPIO_ToggleOutputPin(LED_PORT, LED_PIN);
}

