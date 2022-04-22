/*
 * pwm_test.c
 *
 *  Created on: Apr 21, 2022
 *      Author: atarek
 */

#include <STM32f407xx_PWM.h>
#include <STM32f407xx_TIMER.h>
#include "STM32f407xx_GPIO.h"

#define LED_PORT 				GPIOA
#define LED_PIN 				GPIO_PIN_NO_5

#define BUTTON_PORT 			GPIOC
#define BUTTON_PIN 				GPIO_PIN_NO_13

int main (void)
{
	GPIO_Handle_t GpioLed;

	/*  LED GPIO configuration */
	GpioLed.pGPIOx = LED_PORT;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = LED_PIN;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioLed.GPIO_PinConfig.GPIO_PinAltFunMode = 1;
	GPIO_Init(&GpioLed);

	PWM_Handle_t pwm_handle;
	pwm_handle.pTIMx = TIM2;
	pwm_handle.TIM_Config.PWM_Prescaler = 16;
	pwm_handle.TIM_Config.PWM_AutoRelaod = 1000;
	pwm_handle.TIM_Config.PWM_DutyCycle = 50;
	PWM_init(&pwm_handle);

	uint8_t i = 0;
	while (1)
	{
		uint8_t duty_cyle = (i % 11) * 10;
		PWM_setDutyCyle(&pwm_handle, duty_cyle);
		systickDelayMs(1000);
		i++;
	}
	return 0;
}

