/*
 * STM32f407xx_TIMER.c
 *
 *  Created on: Apr 16, 2022
 *      Author: atarek
 */

/* Timer Types
 * ===========
 * TIM1				TIM_ADVANCED_RegDef_t
 * TIM2				TIM_2_to_5_GENERAL_RegDef_t
 * TIM3				TIM_2_to_5_GENERAL_RegDef_t
 * TIM4				TIM_2_to_5_GENERAL_RegDef_t
 * TIM5				TIM_2_to_5_GENERAL_RegDef_t
 * TIM9				TIM_9_GENERAL_RegDef_t
 * TIM10			TIM_10_11_GENERAL_RegDef_t
 * TIM11			TIM_10_11_GENERAL_RegDef_t
 */
#include <STM32f407xx_TIMER.h>

#define SYSTICK_PER_MILLISECOND (16000 - 1)

/*********************************************************************
 * @fn      		  - systickDelayMs
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
void systickDelayMs(uint32_t delay)
{
	/* Reload the number of clock per millisecond */
	SYSTICK->STK_LOAD = SYSTICK_PER_MILLISECOND;

	/* Clear the current value register */
	SYSTICK->STK_VAL = 0;

	/* Select clock source */
	SYSTICK->STK_CTRL |= (1 << SYSTICK_STK_CTRL_CLKSOURCE);

	/* Enable systick */
	SYSTICK->STK_CTRL |= (CLKSOURCE_AHB << SYSTICK_STK_CTRL_ENABLE);

	for (int i = 0; i < delay; i++)
	{
		/* wait until COUNTFLAG is set */
		while ((SYSTICK->STK_CTRL & (1 << SYSTICK_STK_CTRL_COUNTFLAG)) == 0);
	}

	/* Clear the STK_CTRL register */
	SYSTICK->STK_CTRL = 0;
}

/*********************************************************************
 * @fn      		  - TIM_ADVANCED_PeriClockControl
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
void TIM_ADVANCED_PeriClockControl(TIM_ADVANCED_RegDef_t *pTIMx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pTIMx == TIM1)
		{
			TIM1_PCLK_EN();
		}
	}
	else
	{
		//TODO
	}
}

/*********************************************************************
 * @fn      		  - TIM_2_to_5_GENERAL_PeriClockControl
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
void TIM_2_to_5_GENERAL_PeriClockControl(TIM_2_to_5_GENERAL_RegDef_t *pTIMx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pTIMx == TIM2)
		{
			TIM2_PCLK_EN();
		}else if (pTIMx == TIM3)
		{
			TIM3_PCLK_EN();
		}else if(pTIMx == TIM4)
		{
			TIM4_PCLK_EN();
		}else if (pTIMx == TIM5)
		{
			TIM5_PCLK_EN();
		}
	}
	else
	{
		//TODO
	}
}

/*********************************************************************
 * @fn      		  - TIM_9_GENERAL_PeriClockControl
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
void TIM_9_GENERAL_PeriClockControl(TIM_9_GENERAL_RegDef_t *pTIMx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pTIMx == TIM9)
		{
			TIM9_PCLK_EN();
		}
	}
	else
	{
		//TODO
	}
}

/*********************************************************************
 * @fn      		  - TIM_10_11_GENERAL_PeriClockControl
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
void TIM_10_11_GENERAL_PeriClockControl(TIM_10_11_GENERAL_RegDef_t *pTIMx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pTIMx == TIM10)
		{
			TIM10_PCLK_EN();
		}else if (pTIMx == TIM11)
		{
			TIM11_PCLK_EN();
		}
	}
	else
	{
		//TODO
	}
}

/*********************************************************************
 * @fn      		  - TIM_ADVANCED_init
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
void TIM_ADVANCED_init(TIM_ADVANCED_Handle_t *pTIMHandle)
{
	/* Enable Clock for TIM */
	TIM_ADVANCED_PeriClockControl(pTIMHandle->pTIMx, ENABLE);

	/* Set the prescaler */
	pTIMHandle->pTIMx->TIMx_PSC = pTIMHandle->TIM_Config.TIM_Prescaler - 1;

	/* Set auto reload value */
	pTIMHandle->pTIMx->TIMx_ARR = pTIMHandle->TIM_Config.TIM_AutoRelaod - 1;

	if(pTIMHandle->TIM_Config.TIM_InterruptEnable)
	{
		/* Enable Timer update interrupt */
		pTIMHandle->pTIMx->TIMx_DIER |= (1 << TIM_ADV_DIER_UIF);

		/* Enable Timer update interrupt in NVIC */
		NVIC_ISER->ISER[0] |= (1 << pTIMHandle->TIM_Config.TIM_IRQNumber);
	}
	else
	{
		/* Disable Timer update interrupt */
		pTIMHandle->pTIMx->TIMx_DIER &= ~(1 << TIM_ADV_DIER_UIF);

		/* Disable Timer update interrupt in NVIC */
		NVIC_ICER->ICER[0] &= ~(1 << pTIMHandle->TIM_Config.TIM_IRQNumber);
	}

	/* Enable Timer */
	pTIMHandle->pTIMx->TIMx_CR1 |= (1 << TIM_ADV_CR1_CEN);

}

/*********************************************************************
 * @fn      		  - Check_TIM_ADVANCED_Interrupt_flag
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
uint8_t Check_TIM_ADVANCED_Interrupt_flag(TIM_ADVANCED_Handle_t *pTIMHandle)
{
	if((pTIMHandle->pTIMx->TIMx_SR & (1 << TIM_ADV_SR_UIF)) == 1)
	{
		/* Timer interrupt flag updated */
		pTIMHandle->pTIMx->TIMx_SR &= ~(1 << TIM_ADV_SR_UIF);
		return FLAG_UPDATED;
	}
	else
	{
		/* Timer interrupt flag not updated */
		return FLAG_NOT_UPDATED;
	}
}

/*********************************************************************
 * @fn      		  - TIM_IRQHandling
 *
 * @brief             - This function should be called when an interrupt is triggered
 *
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  none

 */
void TIM_IRQHandling(TIM_ADVANCED_Handle_t *pTIMHandle)
{
	/* Check if the interrupt is pending */
	if(pTIMHandle->pTIMx->TIMx_SR & (1 << TIM_ADV_SR_UIF))
	{
		/* Clear the update interrupt flag */
		pTIMHandle->pTIMx->TIMx_SR &= ~(1 << TIM_ADV_SR_UIF);
	}
}
