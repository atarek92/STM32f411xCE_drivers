/*
 * STM32f407xx_ADC.h
 *
 *  Created on: Apr 14, 2022
 *      Author: atarek
 */

#ifndef INC_STM32F407XX_ADC_H_
#define INC_STM32F407XX_ADC_H_

/*
 *  Configuration structure for SPIx peripheral
 */

#include "stm32f407xx.h"

typedef struct
{
	uint8_t ADC_Channel;			/* ADC Channel */
	uint8_t ADC_SeqOrder;			/* ADC Sequence Order */
	uint8_t ADC_ConverterMode;		/* A/D Converter Mode Single / Continuous */

}ADC_Config_t;

/*
 *Handle structure for ADCx peripheral
 */

typedef struct
{
	ADC_RegDef_t *pADCx;
	ADC_Config_t ADC_Config;

}ADC_Handle_t;


/* @ADC_Channel */

#define ADC_CHANNEL_0             	0
#define ADC_CHANNEL_1             	1
#define ADC_CHANNEL_2             	2
#define ADC_CHANNEL_3             	3
#define ADC_CHANNEL_4             	4
#define ADC_CHANNEL_5             	5
#define ADC_CHANNEL_6             	6
#define ADC_CHANNEL_7             	7
#define ADC_CHANNEL_8             	8
#define ADC_CHANNEL_9             	9
#define ADC_CHANNEL_10             	10
#define ADC_CHANNEL_11             	11
#define ADC_CHANNEL_12             	12
#define ADC_CHANNEL_13             	13
#define ADC_CHANNEL_14             	14
#define ADC_CHANNEL_15             	15

/* @ADC_SeqOrder */

#define ADC_SEQ_ORDER_1        		0
#define ADC_SEQ_ORDER_2            	1
#define ADC_SEQ_ORDER_3          	2
#define ADC_SEQ_ORDER_4    	    	3
#define ADC_SEQ_ORDER_5          	4
#define ADC_SEQ_ORDER_6             5
#define ADC_SEQ_ORDER_7             6
#define ADC_SEQ_ORDER_8             7
#define ADC_SEQ_ORDER_9             8
#define ADC_SEQ_ORDER_10            9
#define ADC_SEQ_ORDER_11            10
#define ADC_SEQ_ORDER_12            11
#define ADC_SEQ_ORDER_13            12
#define ADC_SEQ_ORDER_14            13
#define ADC_SEQ_ORDER_15            14
#define ADC_SEQ_ORDER_16            15

/* @ADC_ConverterMode */

#define ADC_MODE_SINGLE            	0
#define ADC_MODE_CONT             	1


/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void ADC_PeriClockControl(ADC_RegDef_t *pADCx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void ADC_Init(ADC_Handle_t *pSPIHandle);
void ADC_DeInit(ADC_RegDef_t *pADCx);

/*
 * ADC read
 */
uint32_t ADC_ReadData(ADC_Handle_t *pADCHandle);

#endif /* INC_STM32F407XX_ADC_H_ */
