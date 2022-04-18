/*
 * STM32f407xx_SPI.c
 *
 *  Created on: Mar 26, 2022
 *      Author: atarek
 */

#include "STM32f407xx_SPI.h"

/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
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
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
	else
	{
		//TODO
	}
}


/*********************************************************************
 * @fn      		  - SPI_Init
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
void SPI_Init(SPI_Handle_t *pSPIHandle)
{

	//peripheral clock enable

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//first lets configure the SPI_CR1 register

	uint32_t SPI_CR1_temp = 0;

	//1. configure the device mode
	SPI_CR1_temp |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. Configure the bus config
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		/* Bidi mode should be cleared */
		SPI_CR1_temp &= ~( 1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		/* Bidi mode should be set */
		SPI_CR1_temp |= 1 << SPI_CR1_BIDIMODE;
	}
	else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		/* Bidi mode should be cleared */
		SPI_CR1_temp &= ~( 1 << SPI_CR1_BIDIMODE);
		/* RXONLY bit should be set */
		SPI_CR1_temp |= 1 << SPI_CR1_RXONLY;
	}

	//3. Configure the spi serial clock speed (baud rate)
	SPI_CR1_temp |= pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR;

	//4. Configure the DFF
	SPI_CR1_temp |= pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF;

	//5. configure the CPOL
	SPI_CR1_temp |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;

	//6. configure the CPHA
	SPI_CR1_temp |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;

	//7. configure the SSM
	SPI_CR1_temp |= pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->SPI_CR1 = SPI_CR1_temp;
}

/*********************************************************************
 * @fn      		  - SPI_DeInit
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
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
 //todo
}

/*********************************************************************
 * @fn      		  - SPI_GetFlagStatus
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
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName)
{
	if(pSPIx->SPI_SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*********************************************************************
 * @fn      		  - SPI_SendData
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

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while (Len > 0)
	{
		/* 1. Wait until TXE is set */
		while(SPI_GetFlagStatus(pSPIx,( 1 << SPI_SR_TXE))  == FLAG_RESET );

		if ((pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)) == SPI_DFF_8BITS)
		{
			pSPIx->SPI_DR = *pTxBuffer;
			Len --;
			pTxBuffer ++;
		}
		else
		{
				// Not Implemented
		}
	}
}


/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
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
void SPI_PeripheralControl (SPI_RegDef_t *pSPIx, uint8_t enable)
{
	if(enable == ENABLE)
	{
		pSPIx->SPI_CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
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
void SPI_SSIConfig (SPI_RegDef_t *pSPIx, uint8_t enable)
{
	if(enable == ENABLE)
	{
		pSPIx->SPI_CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SSI);
	}
}
