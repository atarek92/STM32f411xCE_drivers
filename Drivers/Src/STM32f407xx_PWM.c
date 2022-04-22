/*
 * STM32f407xx_PWM.c
 *
 *  Created on: Apr 21, 2022
 *      Author: atarek
 */
#include "STM32f407xx_PWM.h"

/*********************************************************************
 * @fn      		  - PWM_init
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
void PWM_init(PWM_Handle_t *pPWMHandle)
{
	/* Enable Clock of the Timer used by PWM */
	TIM2_PCLK_EN();

	/* Set Prescaler Register*/
	pPWMHandle->pTIMx->TIMx_PSC = pPWMHandle->TIM_Config.PWM_Prescaler - 1;

	/* Set Auto Reload Register */
	pPWMHandle->pTIMx->TIMx_ARR = pPWMHandle->TIM_Config.PWM_AutoRelaod - 1;

	/* Reset Counter Timer */
	pPWMHandle->pTIMx->TIMx_CNT = 0;

	/* Enable PWM Mode */
	pPWMHandle->pTIMx->TIMx_CCMR1 |= (OC1M_PWM1 << TIM_GEN_2_5_CCMR1_OC1M);

	/* Enable PWM Channel 1 of this timer*/
	pPWMHandle->pTIMx->TIMx_CCER |= (1 << TIM_GEN_2_5_CCER_CC1E);

	/* Set Duty Cycle */
	pPWMHandle->pTIMx->TIMx_CCR1 = (pPWMHandle->TIM_Config.PWM_AutoRelaod - 1) * (pPWMHandle->TIM_Config.PWM_DutyCycle / 100.0);

	/* Enable Timer used by PWM */
	pPWMHandle->pTIMx->TIMx_CR1 |= (1 << TIM_GEN_2_5_CR1_CEN);
}

/*********************************************************************
 * @fn      		  - PWM_setDutyCyle
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
void PWM_setDutyCyle(PWM_Handle_t *pPWMHandle, uint8_t duty_cycle)
{
	pPWMHandle->pTIMx->TIMx_CCR1 = (pPWMHandle->TIM_Config.PWM_AutoRelaod - 1) * (duty_cycle / 100.0);

}
