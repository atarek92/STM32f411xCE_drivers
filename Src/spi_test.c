/*
 * spi_test.c
 *
 *  Created on: Apr 3, 2022
 *      Author: atarek
 */

#include "STM32f407xx_GPIO.h"
#include "STM32f407xx_SPI.h"
#include "string.h"

void SPI2_GPIO_Init()
{
	GPIO_Handle_t Spi_pins;
	Spi_pins.pGPIOx = GPIOB;
	Spi_pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	Spi_pins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	Spi_pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	Spi_pins.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_NO_PUPD;
	Spi_pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	/* SCLK */
	Spi_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&Spi_pins);

	/* MOSI */
	Spi_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&Spi_pins);

	/* MISO */
	Spi_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&Spi_pins);

	/* NSS */
	Spi_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&Spi_pins);
}

void SPI2_Init(void)
{
	SPI_Handle_t SPIHandle;
	SPIHandle.pSPIx = SPI2;
	SPIHandle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPIHandle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPIHandle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPIHandle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPIHandle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPIHandle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPIHandle.SPI_Config.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPIHandle);
}


int main (void)
{
	char user_data[] = "Hello World";

	SPI2_GPIO_Init();

	SPI2_Init();

	SPI_SSIConfig(SPI2, ENABLE);

	SPI_PeripheralControl(SPI2, ENABLE);

	SPI_SendData(SPI2, (uint8_t*) user_data, strlen(user_data));

	SPI_PeripheralControl(SPI2, DISABLE);

	return 0;
}
