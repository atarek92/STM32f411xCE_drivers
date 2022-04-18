/*
 * STM32f407xx_SPI.h
 *
 *  Created on: Mar 26, 2022
 *      Author: atarek
 */

#ifndef INC_STM32F407XX_SPI_H_
#define INC_STM32F407XX_SPI_H_

/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Feb 9, 2019
 *      Author: afahmy
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

/*
 *  Configuration structure for SPIx peripheral
 */

typedef struct
{
	uint8_t SPI_DeviceMode;			/* Device Mode */
	uint8_t SPI_BusConfig;			/* Bus Configuration */
	uint8_t SPI_SclkSpeed;			/* Clock Speed */
	uint8_t SPI_DFF;				/* Data frame format */
	uint8_t SPI_CPOL;				/* Clock polarity */
	uint8_t SPI_CPHA;				/* Clock phase */
	uint8_t SPI_SSM;				/* Software slave management */

}SPI_Config_t;

/*
 *Handle structure for SPIx peripheral
 */

typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPI_Config;

}SPI_Handle_t;

/*
 * SPI application states
 */
#define SPI_READY 					0
#define SPI_BUSY_IN_RX 				1
#define SPI_BUSY_IN_TX 				2

/*
 * Possible SPI Application events
 */
#define SPI_EVENT_TX_CMPLT   1
#define SPI_EVENT_RX_CMPLT   2
#define SPI_EVENT_OVR_ERR    3
#define SPI_EVENT_CRC_ERR    4



/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER    1
#define SPI_DEVICE_MODE_SLAVE     0


/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD                1
#define SPI_BUS_CONFIG_HD                2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY    3

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2             	0
#define SPI_SCLK_SPEED_DIV4             	1
#define SPI_SCLK_SPEED_DIV8             	2
#define SPI_SCLK_SPEED_DIV16             	3
#define SPI_SCLK_SPEED_DIV32             	4
#define SPI_SCLK_SPEED_DIV64             	5
#define SPI_SCLK_SPEED_DIV128             	6
#define SPI_SCLK_SPEED_DIV256             	7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS 	0
#define SPI_DFF_16BITS  1

/*
 * @CPOL
 */
#define SPI_CPOL_HIGH 1
#define SPI_CPOL_LOW 0

/*
 * @CPHA
 */
#define SPI_CPHA_HIGH 1
#define SPI_CPHA_LOW 0

/*
 * @SPI_SSM
 */
#define SPI_SSM_EN     1
#define SPI_SSM_DI     0


/*
 * SPI related status flags definitions
 */




/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);


/*
 * IRQ Configuration and ISR handling
 */

/*
 * Other Peripheral Control APIs
 */
 void SPI_PeripheralControl (SPI_RegDef_t *pSPIx, uint8_t enable);
 void SPI_SSIConfig (SPI_RegDef_t *pSPIx, uint8_t enable);

/*
 * Application callback
 */


#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */


#endif /* INC_STM32F407XX_SPI_H_ */
