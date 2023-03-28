/*
 * spi_blocking_tx_testing.c
 *
 *  Created on: 08-Mar-2023
 *      Author: eapple
 */

/*
 * PB12    SPI2_NSS
 * PB13   SPI2_SCK
 * PB14   SPI2_MISO
 * PB15   SPI2_MOSE
 *
 * Alternate function mode AF5
 */

#include "string.h"
#include "stm32f407xx.h"

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);


}

void SPI2_Inits(void)
{

	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN; //software slave management enabled for NSS pin

	SPI_Init(&SPI2handle);
}

int main(void)
{

	SPI2_GPIOInits();

	// This make the NSS signal internally high thus avoid MODE F errir

	SPI2_Inits();

	SPI_SSIConfig(SPI2,ENABLE);
	SPI_PeripheralControl(SPI2,ENABLE);


	char data[] = "Hello Embedded World";

	SPI_SendData(SPI2,(uint8_t *)data,strlen(data));

	while((SPI2->SR & (1 << SPI_SR_BSY)));

	SPI_PeripheralControl(SPI2,DISABLE);

	while(1);

	return 0;
}
