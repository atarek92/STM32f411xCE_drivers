/*
 * STM32f407xx.h
 *
 *  Created on: Mar 13, 2022
 *      Author: atarek
 */

#ifndef STM32F407XX_H_
#define STM32F407XX_H_

#include<stdint.h>

#define __vo volatile

#define NO_BITS_IN_BYTE			8


/******************************************************************************************
 * Base Addresses
 ******************************************************************************************/
/*
 * base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR						(0x08000000U)
#define SRAM1_BASEADDR						(0x20000000U)
#define SRAM 								(SRAM1_BASEADDR)

/*
 * Base addresses of Cortex M4
 */
#define SYSTICK_BASEADDR					(0xE000E010U)
#define NVIC_ISER_BASEADDR					(0xE000E100U)
#define NVIC_ICER_BASEADDR					(0xE000E180U)

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define APB1PERIPH_BASEADDR					(0x40000000U)
#define APB2PERIPH_BASEADDR					(0x40010000U)
#define AHB1PERIPH_BASEADDR					(0x40020000U)
#define AHB2PERIPH_BASEADDR					(0x50000000U)

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOH_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1C00)
#define RCC_BASEADDR						(AHB1PERIPH_BASEADDR + 0x3800)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */
#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400)

#define TIM2_BASEADDR						(APB1PERIPH_BASEADDR + 0x0000)
#define TIM3_BASEADDR						(APB1PERIPH_BASEADDR + 0x0400)
#define TIM4_BASEADDR						(APB1PERIPH_BASEADDR + 0x0800)
#define TIM5_BASEADDR						(APB1PERIPH_BASEADDR + 0x0C00)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */
#define SPI1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3000)

#define SYSCFG_BASEADDR        				(APB2PERIPH_BASEADDR + 0x3800)

#define USART1_BASEADDR						(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR						(APB2PERIPH_BASEADDR + 0x1400)

#define ADC_BASEADDR						(APB2PERIPH_BASEADDR + 0x2000)
#define ADC1_BASEADDR						(ADC_BASEADDR + 0x0000)
#define ADC_COM_REG_BASEADDR				(ADC_BASEADDR + 0x0300)

#define TIM1_BASEADDR						(APB2PERIPH_BASEADDR + 0x0000)
#define TIM9_BASEADDR						(APB2PERIPH_BASEADDR + 0x4000)
#define TIM10_BASEADDR						(APB2PERIPH_BASEADDR + 0x4400)
#define TIM11_BASEADDR						(APB2PERIPH_BASEADDR + 0x4800)

#define EXTI_BASEADDR						(APB2PERIPH_BASEADDR + 0x3C00)


/******************************************************************************************
 * Peripheral register definition structures
 ******************************************************************************************/

/*
 * peripheral register definition structure for GPIOs
 */

typedef struct
{
	__vo uint32_t MODER;						/*!< GPIO port mode register,                    	Address offset: 0x00      */
	__vo uint32_t OTYPER;						/*!< GPIO port output type register,     			Address offset: 0x04      */
	__vo uint32_t OSPEEDR;						/*!< GPIO port output speed register,     			Address offset: 0x08      */
	__vo uint32_t PUPDR;						/*!< GPIO port pull-up/pull-down register,   		Address offset: 0x0C      */
	__vo uint32_t IDR;							/*!< GPIO port input data register,     			Address offset: 0x10      */
	__vo uint32_t ODR;							/*!< GPIO port output data register,     			Address offset: 0x14      */
	__vo uint32_t BSRR;							/*!< GPIO port bit set/reset register,     			Address offset: 0x18      */
	__vo uint32_t LCKR;							/*!< GPIO port configuration lock register,    		Address offset: 0x1C      */
	__vo uint32_t AFR[2];						/*!< AFR[0] : GPIO alternate function low register, AF[1] : GPIO alternate function high register    		Address offset: 0x20-0x24 */
}GPIO_RegDef_t;

/*
 * peripheral register definition structure for RCC
 */
typedef struct
{
  __vo uint32_t CR;            /*!< clock control register,  						Address offset: 0x00 */
  __vo uint32_t PLLCFGR;       /*!< PLL configuration register,     				Address offset: 0x04 */
  __vo uint32_t CFGR;          /*!< clock configuration register,     				Address offset: 0x08 */
  __vo uint32_t CIR;           /*!< clock interrupt register,     					Address offset: 0x0C */
  __vo uint32_t AHB1RSTR;      /*!< AHB1 peripheral reset register,     			Address offset: 0x10 */
  __vo uint32_t AHB2RSTR;      /*!< AHB2 peripheral reset register,     			Address offset: 0x14 */
  uint32_t      RESERVED0[2];  /*!< Reserved, 0x18-0x1C                                                  */
  __vo uint32_t APB1RSTR;      /*!< APB1 peripheral reset register,     			Address offset: 0x20 */
  __vo uint32_t APB2RSTR;      /*!< APB2 peripheral reset register,     			Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                  */
  __vo uint32_t AHB1ENR;       /*!< AHB1 peripheral clock enable register,  		Address offset: 0x30 */
  __vo uint32_t AHB2ENR;       /*!< AHB2 peripheral clock enable register,  		Address offset: 0x34 */
  uint32_t      RESERVED2[2];  /*!< Reserved, 0x38-0x3C                                                  */
  __vo uint32_t APB1ENR;       /*!< APB1 peripheral clock enable register,     		Address offset: 0x40 */
  __vo uint32_t APB2ENR;       /*!< APB2 peripheral clock enable register,  		Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                  */
  __vo uint32_t AHB1LPENR;     /*!< AHB1 periph clk enable in low power mode reg,	Address offset: 0x50 */
  __vo uint32_t AHB2LPENR;     /*!< AHB2 periph clk enable in low power mode reg,	Address offset: 0x54 */
  uint32_t      RESERVED4[2];  /*!< Reserved, 0x58-0x5C                                                  */
  __vo uint32_t APB1LPENR;     /*!< APB1 periph clk enable in low power mode reg,     										Address offset: 0x60 */
  __vo uint32_t APB2LPENR;     /*!< APB2 periph clk enable in low power mode reg,     										Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                  */
  __vo uint32_t BDCR;          /*!< Backup domain control register,     			Address offset: 0x70 */
  __vo uint32_t CSR;           /*!<  clock control & status register,     			Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                  */
  __vo uint32_t SSCGR;         /*!< spread spectrum clock generation register,    	Address offset: 0x80 */
  __vo uint32_t PLLI2SCFGR;    /*!< PLLI2S configuration register,     				Address offset: 0x84 */
  uint32_t      RESERVED7;     /*!< Reserved, 0x88		                                                 */
  __vo uint32_t DCKCFGR;       /*!< Dedicated Clocks Configuration Register,   		Address offset: 0x8C */
} RCC_RegDef_t;

/*
 * peripheral register definition structure for SYSTICK
 */
typedef struct
{
	__vo uint32_t STK_CTRL;						/*!< SysTick control and status register,          	Address offset: 0x00      */
	__vo uint32_t STK_LOAD;						/*!< SysTick reload value register,     			Address offset: 0x04      */
	__vo uint32_t STK_VAL;						/*!< SysTick current value register,     			Address offset: 0x08      */
	__vo uint32_t STK_CALIB;					/*!< SysTick calibration value register,   			Address offset: 0x0C      */
}SYSTICK_RegDef_t;

/*
 * peripheral register definition structure for EXTI
 */

typedef struct
{
	__vo uint32_t IMR;						/*!< Interrupt mask register,          					Address offset: 0x00      */
	__vo uint32_t EMR;						/*!< Event mask register,     							Address offset: 0x04      */
	__vo uint32_t RTSR;						/*!< Rising trigger selection register,     			Address offset: 0x08      */
	__vo uint32_t FTSR;						/*!< Falling trigger selection register,   				Address offset: 0x0C      */
	__vo uint32_t SWIER;					/*!< Software interrupt event register,   				Address offset: 0x10      */
	__vo uint32_t PR;						/*!< Pending register,   								Address offset: 0x14      */
}EXTI_RegDef_t;

/*
 * peripheral register definition structure for SPI
 */

typedef struct
{
	__vo uint32_t  SPI_CR1;		/* SPI control register 1							Address offset: 0x00 */
	__vo uint32_t  SPI_CR2;		/* SPI control register 2							Address offset: 0x04 */
	__vo uint32_t  SPI_SR;		/* SPI status register								Address offset: 0x08 */
	__vo uint32_t  SPI_DR;		/* SPI data register								Address offset: 0x0C */
	__vo uint32_t  SPI_CRCPR;	/* SPI CRC polynomial register						Address offset: 0x10 */
	__vo uint32_t  SPI_RXCRCR;	/* SPI RX CRC register								Address offset: 0x14 */
	__vo uint32_t  SPI_TXCRCR;	/* SPI TX CRC register								Address offset: 0x18 */
	__vo uint32_t  SPI_I2SCFGR;	/* SPI_I2S configuration register					Address offset: 0x1C */
	__vo uint32_t  SPI_I2SPR;	/* SPI_I2S prescaler register						Address offset: 0x20 */
} SPI_RegDef_t;

/*
 * peripheral register definition structure for SYSCFG
 */

typedef struct
{
	__vo uint32_t  MEMRMP;		/* SYSCFG memory remap registeR							Address offset: 0x00 */
	__vo uint32_t  PMC;			/* SYSCFG peripheral mode configuration register		Address offset: 0x04 */
	__vo uint32_t  EXTICR[4];	/* SYSCFG external interrupt configuration register		Address offset: 0x08 - 0x14 */
	__vo uint32_t  RESERVED[2];	/* 														Address offset: 0x18 - 0x1C */
	__vo uint32_t  CMPCR;		/* Compensation cell control register					Address offset: 0x20 */
} SYSCFG_RegDef_t;

/*
 * peripheral register definition structure for I2C
 */


/*
 * peripheral register definition structure for USART
 */

/*
 * peripheral register definition structure for ADC
 */

typedef struct
{
	__vo uint32_t  ADC_SR;		/* ADC status register								Address offset: 0x00 */
	__vo uint32_t  ADC_CR1;		/* ADC control register 1							Address offset: 0x04 */
	__vo uint32_t  ADC_CR2;		/* ADC control register 2							Address offset: 0x08 */
	__vo uint32_t  ADC_SMPR1;	/* ADC sample time register 1						Address offset: 0x0C */
	__vo uint32_t  ADC_SMPR2;	/* ADC sample time register 2						Address offset: 0x10 */
	__vo uint32_t  ADC_JOFR1;	/* ADC injected channel data offset register 1		Address offset: 0x14 */
	__vo uint32_t  ADC_JOFR2;	/* ADC injected channel data offset register 2		Address offset: 0x18 */
	__vo uint32_t  ADC_JOFR3;	/* ADC injected channel data offset register 3		Address offset: 0x1C */
	__vo uint32_t  ADC_JOFR4;	/* ADC injected channel data offset register 4		Address offset: 0x20 */
	__vo uint32_t  ADC_HTR;		/* ADC watchdog higher threshold register			Address offset: 0x24 */
	__vo uint32_t  ADC_LTR;		/* ADC watchdog lower threshold register			Address offset: 0x28 */
	__vo uint32_t  ADC_SQR1;	/* ADC regular sequence register 1					Address offset: 0x2C */
	__vo uint32_t  ADC_SQR2;	/* ADC regular sequence register 2					Address offset: 0x30 */
	__vo uint32_t  ADC_SQR3;	/* ADC regular sequence register 3					Address offset: 0x34 */
	__vo uint32_t  ADC_JSQR;	/* ADC injected sequence register					Address offset: 0x38 */
	__vo uint32_t  ADC_JDR1;	/* ADC injected data register 1						Address offset: 0x3C */
	__vo uint32_t  ADC_JDR2;	/* ADC injected data register 2						Address offset: 0x40 */
	__vo uint32_t  ADC_JDR3;	/* ADC injected data register 3						Address offset: 0x44 */
	__vo uint32_t  ADC_JDR4;	/* ADC injected data register 4						Address offset: 0x48 */
	__vo uint32_t  ADC_DR;		/* ADC regular data register						Address offset: 0x4C */
} ADC_RegDef_t;


typedef struct
{
	uint32_t       RESERVED0;       /* Reserved										Address offset: 0x00 */
	__vo uint32_t  ADC_CCR;		/* ADC common control register						Address offset: 0x04 */
} ADC_Comm_RegDef_t;

/*
 * peripheral register definition structure for TIMER
 */
typedef struct
{
	__vo uint32_t  TIMx_CR1;		/* Control register 1							Address offset: 0x0 */
	__vo uint32_t  TIMx_CR2;		/* Control register 2							Address offset: 0x04 */
	__vo uint32_t  TIMx_SMCR;		/* Slave mode control register					Address offset: 0x08 */
	__vo uint32_t  TIMx_DIER;		/* DMA/iInterrupt enable register 				Address offset: 0x0C */
	__vo uint32_t  TIMx_SR;			/* Status register 								Address offset: 0x10 */
	__vo uint32_t  TIMx_EGR;		/* Event generation register 					Address offset: 0x14 */
	__vo uint32_t  TIMx_CCMR1;      /* Capture/compare mode register 1				Address offset: 0x18 */
	__vo uint32_t  TIMx_CCMR2;      /* Capture/compare mode register 2				Address offset: 0x1C */
	__vo uint32_t  TIMx_CCER;       /* Capture/compare enable register				Address offset: 0x20 */
	__vo uint32_t  TIMx_CNT;		/* Counter 										Address offset: 0x24 */
	__vo uint32_t  TIMx_PSC;		/* Prescaler 									Address offset: 0x28 */
	__vo uint32_t  TIMx_ARR;		/* Auto-reload register 						Address offset: 0x2C */
	uint32_t       RESERVED0;       /* Reserved										Address offset: 0x30 */
	__vo uint32_t  TIMx_CCR1;		/* Capture/compare register 1 					Address offset: 0x34 */
	__vo uint32_t  TIMx_CCR2;		/* Capture/compare register 2 					Address offset: 0x38 */
	__vo uint32_t  TIMx_CCR3;		/* Capture/compare register 3 					Address offset: 0x3C */
	__vo uint32_t  TIMx_CCR4;		/* Capture/compare register 4 					Address offset: 0x40 */
	uint32_t       RESERVED1;       /* Reserved										Address offset: 0x44 */
	__vo uint32_t  TIMx_DCR;		/* DMA control register		 					Address offset: 0x48 */
	__vo uint32_t  TIMx_DMAR;		/* 8 DMA address for full transfer				Address offset: 0x4C */
	__vo uint32_t  TIMx_OR;			/* Option register 1 							Address offset: 0x50 */
} TIM_2_to_5_GENERAL_RegDef_t;

typedef struct
{
	__vo uint32_t  TIMx_CR1;		/* Control register 1							Address offset: 0x0 */
	uint32_t  	   RESERVED0;		/* Reserved										Address offset: 0x04 */
	__vo uint32_t  TIMx_SMCR;		/* Slave mode control register					Address offset: 0x08 */
	__vo uint32_t  TIMx_DIER;		/* Interrupt enable register 					Address offset: 0x0C */
	__vo uint32_t  TIMx_SR;			/* Status register 								Address offset: 0x10 */
	__vo uint32_t  TIMx_EGR;		/* Event generation register 					Address offset: 0x14 */
	__vo uint32_t  TIMx_CCMR1;      /* Capture/compare mode register 1				Address offset: 0x18 */
	uint32_t       RESERVED2;       /* Reserved										Address offset: 0x1C */
	__vo uint32_t  TIMx_CCER;       /* Capture/compare enable register				Address offset: 0x20 */
	__vo uint32_t  TIMx_CNT;		/* Counter 										Address offset: 0x24 */
	__vo uint32_t  TIMx_PSC;		/* Prescaler 									Address offset: 0x28 */
	__vo uint32_t  TIMx_ARR;		/* Auto-reload register 						Address offset: 0x2C */
	uint32_t       RESERVED3;       /* Reserved										Address offset: 0x30 */
	__vo uint32_t  TIMx_CCR1;		/* Capture/compare register 1 					Address offset: 0x34 */
	__vo uint32_t  TIMx_CCR2;		/* Capture/compare register 2					Address offset: 0x38 */
} TIM_9_GENERAL_RegDef_t;

typedef struct
{
	__vo uint32_t  TIMx_CR1;		/* Control register 1							Address offset: 0x0 */
	uint32_t  	   RESERVED0;		/* Reserved										Address offset: 0x04 */
	uint32_t  	   RESERVED1;		/* Reserved										Address offset: 0x08 */
	__vo uint32_t  TIMx_DIER;		/* Interrupt enable register 					Address offset: 0x0C */
	__vo uint32_t  TIMx_SR;			/* Status register 								Address offset: 0x10 */
	__vo uint32_t  TIMx_EGR;		/* Event generation register 					Address offset: 0x14 */
	__vo uint32_t  TIMx_CCMR1;      /* Capture/compare mode register 1				Address offset: 0x18 */
	uint32_t       RESERVED2;       /* Reserved										Address offset: 0x1C */
	__vo uint32_t  TIMx_CCER;       /* Capture/compare enable register				Address offset: 0x20 */
	__vo uint32_t  TIMx_CNT;		/* Counter 										Address offset: 0x24 */
	__vo uint32_t  TIMx_PSC;		/* Prescaler 									Address offset: 0x28 */
	__vo uint32_t  TIMx_ARR;		/* Auto-reload register 						Address offset: 0x2C */
	uint32_t       RESERVED3;       /* Reserved										Address offset: 0x30 */
	__vo uint32_t  TIMx_CCR1;		/* Capture/compare register 1 					Address offset: 0x34 */
	uint32_t       RESERVED4[6];    /* Reserved							 	 Address offset: 0x38 - 0x4C */
	__vo uint32_t  TIMx_OR;			/* Option register 1 							Address offset: 0x50 */
} TIM_10_11_GENERAL_RegDef_t;

typedef struct
{
	__vo uint32_t  TIMx_CR1;		/* Control register 1							Address offset: 0x0 */
	__vo uint32_t  TIMx_CR2;		/* Control register 2							Address offset: 0x04 */
	__vo uint32_t  TIMx_SMCR;		/* Slave mode control register					Address offset: 0x08 */
	__vo uint32_t  TIMx_DIER;		/* DMA/iInterrupt enable register 				Address offset: 0x0C */
	__vo uint32_t  TIMx_SR;			/* Status register 								Address offset: 0x10 */
	__vo uint32_t  TIMx_EGR;		/* Event generation register 					Address offset: 0x14 */
	__vo uint32_t  TIMx_CCMR1;      /* Capture/compare mode register 1				Address offset: 0x18 */
	__vo uint32_t  TIMx_CCMR2;      /* Capture/compare mode register 2				Address offset: 0x1C */
	__vo uint32_t  TIMx_CCER;       /* Capture/compare enable register				Address offset: 0x20 */
	__vo uint32_t  TIMx_CNT;		/* Counter 										Address offset: 0x24 */
	__vo uint32_t  TIMx_PSC;		/* Prescaler 									Address offset: 0x28 */
	__vo uint32_t  TIMx_ARR;		/* Auto-reload register 						Address offset: 0x2C */
	__vo uint32_t  TIMx_RCR;        /* Repetition counter register					Address offset: 0x30 */
	__vo uint32_t  TIMx_CCR1;		/* Capture/compare register 1 					Address offset: 0x34 */
	__vo uint32_t  TIMx_CCR2;		/* Capture/compare register 2 					Address offset: 0x38 */
	__vo uint32_t  TIMx_CCR3;		/* Capture/compare register 3 					Address offset: 0x3C */
	__vo uint32_t  TIMx_CCR4;		/* Capture/compare register 4 					Address offset: 0x40 */
	__vo uint32_t  TIMx_BDTR;		/* Break and dead-time register 				Address offset: 0x44 */
	__vo uint32_t  TIMx_DCR;		/* DMA control register		 					Address offset: 0x48 */
	__vo uint32_t  TIMx_DMAR;		/* 8 DMA address for full transfer				Address offset: 0x4C */

} TIM_ADVANCED_RegDef_t;

typedef struct
{
	__vo uint32_t  ISER[8];		/* Interrupt set-enable register				Address offset: 0x0 */
} NVIC_ISER_RegDef_t;

typedef struct
{
	__vo uint32_t  ICER[8];		/* Interrupt clear-enable register				Address offset: 0x0 */
} NVIC_ICER_RegDef_t;

/******************************************************************************************
 * Peripheral definitions (Peripheral base addresses type-casted to xxx_RegDef_t)
 ******************************************************************************************/

#define GPIOA  				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB  				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC  				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD  				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE  				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH  				((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC 				((RCC_RegDef_t*)RCC_BASEADDR)
#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)
#define SYSTICK				((SYSTICK_RegDef_t*)SYSTICK_BASEADDR)
#define NVIC_ISER			((NVIC_ISER_RegDef_t*)NVIC_ISER_BASEADDR)
#define NVIC_ICER			((NVIC_ICER_RegDef_t*)NVIC_ICER_BASEADDR)

#define SPI1  				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2  				((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3  				((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1  				((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2  				((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3  				((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1  			((USART_RegDef_t*)USART1_BASEADDR)
#define USART2  			((USART_RegDef_t*)USART2_BASEADDR)
#define USART6  			((USART_RegDef_t*)USART6_BASEADDR)

#define ADC1  				((ADC_RegDef_t*)ADC1_BASEADDR)
#define ADC_COMM  			((ADC_Comm_RegDef_t*)ADC_COM_REG_BASEADDR)

#define TIM1				((TIM_ADVANCED_RegDef_t*)TIM1_BASEADDR)
#define TIM2				((TIM_2_to_5_GENERAL_RegDef_t*)TIM2_BASEADDR)
#define TIM3				((TIM_2_to_5_GENERAL_RegDef_t*)TIM3_BASEADDR)
#define TIM4				((TIM_2_to_5_GENERAL_RegDef_t*)TIM4_BASEADDR)
#define TIM5				((TIM_2_to_5_GENERAL_RegDef_t*)TIM5_BASEADDR)
#define TIM9				((TIM_9_GENERAL_RegDef_t*)TIM9_BASEADDR)
#define TIM10				((TIM_10_11_GENERAL_RegDef_t*)TIM10_BASEADDR)
#define TIM11				((TIM_10_11_GENERAL_RegDef_t*)TIM11_BASEADDR)


#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)

/******************************************************************************************
 * Clock Enable / Disable Macros
 ******************************************************************************************/

/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()    	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))

/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))

/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN() (RCC->APB2ENR |= (1 << 20))

/*
 * Clock Enable Macros for ADCx peripherals
 */
#define ADC1_PCLK_EN() (RCC->APB2ENR |= (1 << 8))

/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART6_PCLK_EN() (RCC->APB2ENR |= (1 << 5))

/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))

/*
 * Clock Enable Macros for TIMERS peripherals
 */
#define TIM1_PCLK_EN() (RCC->APB2ENR |= (1 << 0))
#define TIM2_PCLK_EN() (RCC->APB1ENR |= (1 << 0))
#define TIM3_PCLK_EN() (RCC->APB1ENR |= (1 << 1))
#define TIM4_PCLK_EN() (RCC->APB1ENR |= (1 << 2))
#define TIM5_PCLK_EN() (RCC->APB1ENR |= (1 << 3))
#define TIM9_PCLK_EN() (RCC->APB2ENR |= (1 << 16))
#define TIM10_PCLK_EN() (RCC->APB2ENR |= (1 << 17))
#define TIM11_PCLK_EN() (RCC->APB2ENR |= (1 << 18))

/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()

/*
 * Clock Disable Macros for SPIx peripherals
 */

/*
 * Clock Disable Macros for USARTx peripherals
 */


/*
 * Clock Disable Macros for SYSCFG peripheral
 */

/*
 *  Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOH_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)

/******************************************************************************************
 * Generic macros
 ******************************************************************************************/

#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET        SET
#define GPIO_PIN_RESET      RESET
#define FLAG_SET 			SET
#define FLAG_RESET			RESET
#define FLAG_UPDATED		SET
#define FLAG_NOT_UPDATED	RESET


/******************************************************************************************
 *Bit position definitions of SYSTICK
 ******************************************************************************************/
/*
 * Bit position definitions STK_CTRL
 */
#define SYSTICK_STK_CTRL_ENABLE  		 0				/* Counter enable */
#define SYSTICK_STK_CTRL_TICKINT  		 1				/* SysTick exception request enable */
#define SYSTICK_STK_CTRL_CLKSOURCE  	 2				/* Clock source selection */
#define SYSTICK_STK_CTRL_COUNTFLAG  	 16				/* Counter Flag "Returns 1 if timer counted to 0 since last time this was read" */

/*
 * Bit position definitions STK_LOAD
 */
#define SYSTICK_STK_LOAD_RELOAD  		 0				/* RELOAD value */

/*
 * Bit position definitions STK_VAL
 */
#define SYSTICK_STK_VAL_CURRENT  		 0				/* Current counter value */

/*
 * Bit position definitions STK_CALIB
 */
#define SYSTICK_STK_CALIB_TENMS  		 0				/* Calibration value */
#define SYSTICK_STK_CALIB_SKEW  		 30				/* SKEW flag */
#define SYSTICK_STK_CALIB_NOREF  		 31				/* NOREF flag */

/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0				/* Clock phase */
#define SPI_CR1_CPOL      				 1				/* Clock polarity */
#define SPI_CR1_MSTR     				 2				/* Master selection */
#define SPI_CR1_BR   					 3				/* Baud rate control */
#define SPI_CR1_SPE     				 6				/* SPI enable */
#define SPI_CR1_LSBFIRST   			 	 7				/* Frame format */
#define SPI_CR1_SSI     				 8				/* Internal slave select */
#define SPI_CR1_SSM      				 9				/* Software slave management */
#define SPI_CR1_RXONLY      		 	10				/* Receive only */
#define SPI_CR1_DFF     			 	11				/* Data frame format */
#define SPI_CR1_CRCNEXT   			 	12				/* CRC transfer next */
#define SPI_CR1_CRCEN   			 	13				/* Hardware CRC calculation enable */
#define SPI_CR1_BIDIOE     			 	14				/* Output enable in bidirectional mode */
#define SPI_CR1_BIDIMODE      			15				/* Bidirectional data mode enable */

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0				/* Rx buffer DMA enable */
#define SPI_CR2_TXDMAEN				 	1				/* Tx buffer DMA enable */
#define SPI_CR2_SSOE				 	2				/* SS output enable */
#define SPI_CR2_FRF						4				/* Frame format */
#define SPI_CR2_ERRIE					5				/* Error interrupt enable */
#define SPI_CR2_RXNEIE				 	6				/* RX buffer not empty interrupt enable */
#define SPI_CR2_TXEIE					7				/* Tx buffer empty interrupt enable
 */


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0				/* Receive buffer not empty */
#define SPI_SR_TXE				 		1				/* Transmit buffer empty */
#define SPI_SR_CHSIDE				 	2				/* Channel side */
#define SPI_SR_UDR					 	3				/* Underrun flag */
#define SPI_SR_CRCERR				 	4				/* CRC error flag */
#define SPI_SR_MODF					 	5				/* Mode fault */
#define SPI_SR_OVR					 	6				/* Overrun flag */
#define SPI_SR_BSY					 	7				/* Busy flag */
#define SPI_SR_FRE					 	8				/* Frame format error */

/******************************************************************************************
 *Bit position definitions of I2C peripheral
 ******************************************************************************************/

/*
 * Bit position definitions I2C_CR1
 */

/*
 * Bit position definitions I2C_CR2
 */

/*
 * Bit position definitions I2C_OAR1
 */

/*
 * Bit position definitions I2C_SR1
 */

/*
 * Bit position definitions I2C_SR2
 */

/*
 * Bit position definitions I2C_CCR
 */


/******************************************************************************************
 *Bit position definitions of USART peripheral
 ******************************************************************************************/

/*
 * Bit position definitions USART_CR1
 */

/*
 * Bit position definitions USART_CR2
 */

/*
 * Bit position definitions USART_CR3
 */

/*
 * Bit position definitions USART_SR
 */

/******************************************************************************************
 *Bit position definitions of ADC peripheral
 ******************************************************************************************/

/*
 * Bit position definitions ADC_SR
 */
#define ADC_SR_AWD						0				/*  */
#define ADC_SR_EOC						1				/*  */
#define ADC_SR_JEOC						2				/*  */
#define ADC_SR_JSTRT					3				/*  */
#define ADC_SR_STRT						4				/*  */
#define ADC_SR_OVR						6				/*  */

/*
 * Bit position definitions ADC_CR1
 */
#define ADC_CR1_EOCIE     				 5				/* Interrupt enable for end of conversion */

/*
 * Bit position definitions ADC_CR2
 */
#define ADC_CR2_ADON     				 0				/* ADC Converter ON / OFF */
#define ADC_CR2_CONT     				 1				/* ADC Converter Mode Single / Continuous */
#define ADC_CR2_SWSTART     			 30				/* Start conversion of regular channels */

/*
 * Bit position definitions ADC_SMPR1
 */

/*
* Bit position definitions ADC_SQR1
*/
#define ADC_SQR_COV_13     				 0				/* 13th conversion in regular sequence */
#define ADC_SQR_COV_14     				 5				/* 14th conversion in regular sequence */
#define ADC_SQR_COV_15     				 10				/* 15th conversion in regular sequence */
#define ADC_SQR_COV_16     				 15				/* 16th conversion in regular sequence */
#define ADC_SQR_SEQ_LEN    				 20				/* Regular channel sequence length */

/*
* Bit position definitions ADC_SQR2
*/
#define ADC_SQR_COV_7     				 0				/* 7th conversion in regular sequence */
#define ADC_SQR_COV_8     				 5				/* 8th conversion in regular sequence */
#define ADC_SQR_COV_9     				 10				/* 9th conversion in regular sequence */
#define ADC_SQR_COV_10     				 15				/* 10th conversion in regular sequence */
#define ADC_SQR_COV_11    				 20				/* 11th conversion in regular sequence */
#define ADC_SQR_COV_12    				 25				/* 12th conversion in regular sequence */

/*
* Bit position definitions ADC_SQR3
*/
#define ADC_SQR_COV_1     				 0				/* 1th conversion in regular sequence */
#define ADC_SQR_COV_2     				 5				/* 2th conversion in regular sequence */
#define ADC_SQR_COV_3     				 10				/* 3th conversion in regular sequence */
#define ADC_SQR_COV_4     				 15				/* 4th conversion in regular sequence */
#define ADC_SQR_COV_5    				 20				/* 5th conversion in regular sequence */
#define ADC_SQR_COV_6    				 25				/* 6th conversion in regular sequence */


/******************************************************************************************
 *Bit position definitions of Advanced Timer peripheral
 ******************************************************************************************/
/*
 * Bit position definitions TIM_ADV_CR1
 */
#define TIM_ADV_CR1_CEN     				 0				/*  Counter enable */

/*
 * Bit position definitions TIM_ADV_SR
 */
#define TIM_ADV_SR_UIF	     				 0				/*  Counter enable */


/*
 * Bit position definitions TIM_ADV_DIER
 */
#define TIM_ADV_DIER_UIF	  				 0				/*  Update interrupt enable */


/******************************************************************************************
 *Bit position definitions of General Timer 2 to 5 peripheral
 ******************************************************************************************/
/*
 * Bit position definitions TIM_GEN_2_5_CR1
 */
#define TIM_GEN_2_5_CR1_CEN     				 0				/* Counter enable */

 /*
  * Bit position definitions TIM_GEN_2_5_CCER
  */
 #define TIM_GEN_2_5_CCER_CC1E     			 	0				/* Enable Output Capture/Compare 1 (OC1) */

/*
 * Bit position definitions TIM_GEN_2_5_CCER
 */
#define TIM_GEN_2_5_CCMR1_OC1M     			 	4				/* Output compare 1 mode */


/******************************************************************************************
 *IRQ Numbers
 ******************************************************************************************/
#define IRQ_NO_EXTI0							6
#define IRQ_NO_EXTI1							7
#define IRQ_NO_EXTI2							8
#define IRQ_NO_EXTI3							9
#define IRQ_NO_EXTI4							10
#define IRQ_NO_ADC1								18
#define IRQ_NO_EXTI9_5							23
#define IRQ_NO_TIM1_BRK_TIM9					24
#define IRQ_NO_TIM1_UP_TIM10					25
#define IRQ_NO_TIM1_TRG_COM_TIM11				26
#define IRQ_NO_TIM1_CC							27
#define IRQ_NO_EXTI15_10						40
#endif /* STM32F407XX_H_ */
