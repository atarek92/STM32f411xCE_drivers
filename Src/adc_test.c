/*
 * adc_test.c
 *
 *  Created on: Apr 15, 2022
 *      Author: atarek
 */

#include "STM32f407xx_GPIO.h"
#include "STM32f407xx_ADC.h"
#include "string.h"


void ADC_GPIO_Init()
{
	GPIO_Handle_t adc_pins;
	adc_pins.pGPIOx = GPIOA;
	adc_pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;
	adc_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIO_Init(&adc_pins);
}

void ADC1_Init(ADC_Handle_t *pADCHandle)
{
	pADCHandle->pADCx = ADC1;
	pADCHandle->ADC_Config.ADC_Channel = ADC_CHANNEL_4;
	pADCHandle->ADC_Config.ADC_SeqOrder = ADC_SEQ_ORDER_1;
	pADCHandle->ADC_Config.ADC_ConverterMode = ADC_MODE_CONT;

	ADC_Init(pADCHandle);
}

uint32_t sensor_data;

int main (void)
{
    ADC_GPIO_Init();

    ADC_Handle_t ADCHandle;
	ADC1_Init(&ADCHandle);

	while (1)
	{
		sensor_data = ADC_ReadData(&ADCHandle);
	}
	return 0;
}
