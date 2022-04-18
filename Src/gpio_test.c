/*
 * gpio_test.c
 *
 *  Created on: Apr 15, 2022
 *      Author: atarek
 */

#include "STM32f407xx_GPIO.h"

#define LED_PORT 				GPIOA
#define LED_PIN 				GPIO_PIN_NO_5

#define BUTTON_PORT 			GPIOC
#define BUTTON_PIN 				GPIO_PIN_NO_13

#define BTN_PRESSED 			0

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}

int main(void)
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

	while(1)
	{
		if(GPIO_ReadFromInputPin(BUTTON_PORT,BUTTON_PIN) == BTN_PRESSED)
		{
			delay();
			GPIO_ToggleOutputPin(LED_PORT,LED_PIN);
		}
	}

	return 0;
}
