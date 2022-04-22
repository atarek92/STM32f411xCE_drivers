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
#define CLKSOURCE_AHB_DIV_8			0 					/* AHB/8 */
#define CLKSOURCE_AHB				1 					/* Processor clock (AHB) */

/* @Output compare 1 mode */
#define OC1M_Forzen					0 					/* The comparison between the output compare register TIMx_CCR1 and the
														   counter TIMx_CNT has no effect on the outputs.(this mode is used to
														   generate a timing base) */
#define OC1M_Active					1 					/* Set channel 1 to active level on match. OC1REF signal is forced high
														   when the counter TIMx_CNT matches the capture/compare register 1
														   (TIMx_CCR1) */
#define OC1M_Inactive				2 					/* Set channel 1 to inactive level on match. OC1REF signal is forced low
														   when the counter TIMx_CNT matches the capture/compare register 1
														   (TIMx_CCR1) */
#define OC1M_Toggle					3 					/* OC1REF toggles when TIMx_CNT=TIMx_CCR1 */
#define OC1M_ForceInactive			4 					/* Force inactive level - OC1REF is forced low */
#define OC1M_ForceActive			5 					/* Force active level - OC1REF is forced high */
#define OC1M_PWM1					6 					/* In up counting, channel 1 is active as long as TIMx_CNT<TIMx_CCR1
														   else inactive. In down counting, channel 1 is inactive (OC1REF=‘0)
														   as long as TIMx_CNT>TIMx_CCR1 else active (OC1REF=1) */
#define OC1M_PWM2					7 					/* In up counting, channel 1 is inactive as long as TIMx_CNT<TIMx_CCR1
														   else active. In down counting, channel 1 is active as long as
														   TIMx_CNT>TIMx_CCR1 else inactive */

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN 		0

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
