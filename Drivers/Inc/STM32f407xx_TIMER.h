/*
 * STM32f407xx_TIMER.h
 *
 *  Created on: Apr 16, 2022
 *      Author: atarek
 */

#ifndef INC_STM32F407XX_TIMER_H_
#define INC_STM32F407XX_TIMER_H_

#include "stm32f407xx.h"


typedef struct
{
	uint16_t TIM_Prescaler;			/* Timer Prescaler */
	uint16_t TIM_AutoRelaod;		/* Timer auto-reload register */

}TIM_Config_t;

/*
 *Handle structure for TIMx_ADVANCED peripheral
 */

typedef struct
{
	TIM_ADVANCED_RegDef_t *pTIMx;
	TIM_Config_t TIM_Config;

}TIM_ADVANCED_Handle_t;


/* @SYSTICK_CLKSOURCE */
#define CLKSOURCE_AHB_DIV_8					0 					/* AHB/8 */
#define CLKSOURCE_AHB						1 					/* Processor clock (AHB) */

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/

void systickDelayMs(uint32_t delay);

void TIM_ADVANCED_PeriClockControl(TIM_ADVANCED_RegDef_t *pTIMx, uint8_t EnorDi);
void TIM_2_to_5_GENERAL_PeriClockControl(TIM_2_to_5_GENERAL_RegDef_t *pTIMx, uint8_t EnorDi);
void TIM_9_GENERAL_PeriClockControl(TIM_9_GENERAL_RegDef_t *pTIMx, uint8_t EnorDi);
void TIM_10_11_GENERAL_PeriClockControl(TIM_10_11_GENERAL_RegDef_t *pTIMx, uint8_t EnorDi);

void TIM_ADVANCED_init(TIM_ADVANCED_Handle_t *pTIMHandle);
uint8_t Check_TIM_ADVANCED_Interrupt_flag(TIM_ADVANCED_Handle_t *pTIMHandle);


#endif /* INC_STM32F407XX_TIMER_H_ */
