/*
 * STM32f407xx_GPIO.c
 *
 *  Created on: Mar 13, 2022
 *      Author: atarek
 */


#include "STM32f407xx_GPIO.h"

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		//TODO
	}

}


/*********************************************************************
 * @fn      		  - GPIO_Init
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
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t set_value = 0;

	/* Enable the peripheral clock */
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	/* 1 . configure the mode of GPIO pin */
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)			/* Non-interrupt mode */
	{
		set_value = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~ (0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);			/* Clear Register */
		pGPIOHandle->pGPIOx->MODER |= set_value;
	}
	else																		/* Interrupt mode */
	{

	}

	set_value = 0;

	/* 2. Configure the speed */
	set_value = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~ (0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);			/* Clear Register */
	pGPIOHandle->pGPIOx->OSPEEDR |= set_value;

	set_value = 0;

	/* 3. Configure the Push Pull settings */
	set_value = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~ (0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);			/* Clear Register */
	pGPIOHandle->pGPIOx->PUPDR |= set_value;

	set_value = 0;

	/* 4. Configure the optype */
	set_value = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~ (0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);			/* Clear Register */
	pGPIOHandle->pGPIOx->OTYPER |= set_value;

	set_value = 0;

	//5. configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t AFR_register = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / NO_BITS_IN_BYTE;
		uint8_t AFR_pin = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % NO_BITS_IN_BYTE;

		pGPIOHandle->pGPIOx->AFR[AFR_register] &= ~ (0xF << (4 * AFR_pin));							/* Clear Register */
		pGPIOHandle->pGPIOx->AFR[AFR_register] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * AFR_pin);
	}
}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
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
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}

}


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
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
   uint8_t value;

   value = (uint8_t )((pGPIOx->IDR  >> PinNumber) & 0x00000001 ) ;

   return value;
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
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;

	value = (uint16_t)pGPIOx->IDR;

	return value;
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
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{

	if(Value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= ( 1 << PinNumber);
	}else
	{
		//write 0
		pGPIOx->ODR &= ~( 1 << PinNumber);
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
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR  = Value;
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
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR  ^= ( 1 << PinNumber);
}


