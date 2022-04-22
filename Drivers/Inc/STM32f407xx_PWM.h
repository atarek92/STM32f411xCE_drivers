/*
 * STM32f407xx_PWM.h
 *
 *  Created on: Apr 21, 2022
 *      Author: atarek
 */

#ifndef INC_STM32F407XX_PWM_H_
#define INC_STM32F407XX_PWM_H_

#include "STM32f407xx_TIMER.h"


typedef struct
{
	uint16_t PWM_Prescaler;			/* PWM Prescaler */
	uint16_t PWM_AutoRelaod;		/* PWM auto-reload register */
	uint8_t PWM_DutyCycle;			/* PWM Duty Cycle (Percentage) */
}PWM_Config_t;

/*
 *Handle structure for PWM peripheral
 */

typedef struct
{
	TIM_2_to_5_GENERAL_RegDef_t *pTIMx;
	PWM_Config_t TIM_Config;
}PWM_Handle_t;


void PWM_init(PWM_Handle_t *pPWMHandle);
void PWM_setDutyCyle(PWM_Handle_t *pPWMHandle, uint8_t duty_cycle);


#endif /* INC_STM32F407XX_PWM_H_ */
