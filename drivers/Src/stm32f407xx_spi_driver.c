/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: 06-Mar-2023
 *      Author: Wisal Muhammad
 */

#include "stm32f407xx_spi_driver.h"

static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


void SPI_PeripheralControl(SPI_RegDef_t * pSPIx,uint8_t EnOrDi) {

	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnOrDi)
{

	if(EnOrDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}

	} else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}

}

void SPI_SSIConfig(SPI_RegDef_t * pSPIx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}



void SPI_Init(SPI_Handle_t *pHandle)
{

	SPI_PeriClockControl(pHandle->pSPIx,ENABLE);

    // Configure the SPI_CR1 Register
	uint32_t tempRegister = 0;

	// Devide Mode setting
	tempRegister |= pHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	if(pHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// BIDI Mode should be cleared
		tempRegister &= ~(1 << SPI_CR1_BIDI_MODE);

	} else if(pHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// BIDI Mode should be set
		tempRegister |= (1 << SPI_CR1_BIDI_MODE);
	}
	else if(pHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// BIDI mode should be cleared
		// RXOnly bit must be selected then
		tempRegister &= ~(1 << SPI_CR1_BIDI_MODE);
		tempRegister |= (1 << SPI_CR1_RXONLY);
	}

	// Configure the SPI serial clock speed(Baud rate)

	tempRegister |= pHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	// Configure the DFF
	tempRegister |= pHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//Configure the CPOL
	tempRegister |= pHandle->SPIConfig.SPI_CPOL<< SPI_CR1_CPOL;

	// Configure the CPHA
	tempRegister |= pHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	pHandle->pSPIx->CR1 = tempRegister;

}


void SPI_DeInit(SPI_RegDef_t *pSPIx)
{

	if(pSPIx == SPI1)
	{
		SPI1_PCLK_DI();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_PCLK_DI();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_PCLK_DI();
	}

	SPI_PeriClockControl(pSPIx,DISABLE);
	SPI_PeripheralControl(pSPIx,DISABLE);

	// TODO

}

// Blocking call

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t len)
{

	while(len > 0)
	{
		// Wait until TXE bit is set

		while(!(pSPIx->SR & (1 << 1)));

		//Check the DFF bit
		if((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			//16bit mode
			pSPIx->DR = *((uint16_t *)pTxBuffer);
			len -=2;
			(uint16_t *)pTxBuffer++;
		}
		else
		{
			//8 bit mode
			pSPIx->DR = *pTxBuffer;
			len--;
			pTxBuffer++;
		}


	}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t len)
{

	while(len > 0)
		{
			// Wait until TXE bit is set

			while(!(pSPIx->SR & (1 << 0)));

			//Check the DFF bit
			if((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
			{
				//16bit mode
				*((uint16_t *)pRxBuffer) = pSPIx->DR;
				len -=2;
				(uint16_t *)pRxBuffer++;
			}
			else
			{
				//8 bit mode
				*pRxBuffer = pSPIx->DR;
				len--;
				pRxBuffer++;
			}


	}
}

uint8_t SPI_SendData_IT(SPI_Handle_t *pHandle,uint8_t *pTxBuffer,uint32_t len)
{

	uint8_t state = pHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		pHandle->pTxBuffer = pTxBuffer;
		pHandle->TxLen = len;
		pHandle->TxState = SPI_BUSY_IN_TX;
		pHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}

	return state;
}


uint8_t SPI_ReceiveData_IT(SPI_Handle_t *pHandle,uint8_t *pRxBuffer,uint32_t len)
{

   uint8_t state = pHandle->RxState;


   	if(state != SPI_BUSY_IN_RX)
   	{
   		pHandle->pRxBuffer = pRxBuffer;
   		pHandle->RxLen = len;
   		pHandle->RxState = SPI_BUSY_IN_RX;
   		pHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
   	}
   return state;
}


void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi)
	{
		if(IRQNumber <=31)
		{
			// configure ISER0
			*NVIC_ISER0 |= (1 << IRQNumber);

		} else if(IRQNumber > 31 && IRQNumber < 64)
		{
			// configure ISER1
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));

		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			// configure ISER2
			*NVIC_ISER3 |= (1 << (IRQNumber % 64));
		}
	} else
	{
		if(IRQNumber <= 31)
		{
			// configure ISER0
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if(IRQNumber > 31 && IRQNumber < 64)
		{
			// configure ISER1
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));

		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			// configure ISER2
			*NVIC_ICER3 |= (1 << (IRQNumber % 64));
		}
	}

}


void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;  // to find register
	uint8_t iprx_section = IRQNumber % 4; // to find register section

	// to find the address we have to add (iprx + 4)
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx)) |= (IRQPriority << shift_amount);

}


void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

	uint8_t temp1 , temp2;
		//first lets check for TXE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE);

	if( temp1 && temp2)
	{
			//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

		// check for RXNE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);

	if( temp1 && temp2)
	{
			//handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

		// check for ovr flag
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

	if( temp1 && temp2)
	{
			//handle ovr error
		spi_ovr_err_interrupt_handle(pHandle);
	}

}

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
  //TODO
}

static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
  //TODO
}


static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
 //TODO
}



