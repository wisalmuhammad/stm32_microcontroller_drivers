/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: 24-Jan-2023
 *      Author: eapple
 */

#include "stm32f407xx_gpio_driver.h"



void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {


	uint32_t temp = 0; //temp register


	GPIO_PeriClockControl(pGPIOHandle->pGPIOx,ENABLE);

	//printf("GPIO_MODE Set  MODE : %d, PIN No: %d \n",pGPIOHandle->GPIO_PinConfig.GPIO_PinMode,pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{

			//the non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //setting

	} else {
		// configuring the interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// configure FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear RTST
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		} else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// configure RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear FTST
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// configure both FTSR and RTSR
			// configure RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear RTST
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2 configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portCode << (temp2 * 4);

		//3 enable the EXTI interrupt delivery using IMR

		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;

	}

	//2. configure the speed
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
    pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

		//3. configure the pupd settings
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;


		//4. configure the optype
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;

		//5. configure the alt functionality
	temp = 0;
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
			//configure the alt function registers.

		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber  % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ) ); //clearing

		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 ) );
	}

}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {

	if(pGPIOx == GPIOA) {
		GPIOA_PCLK_EN();
	}
	else if(pGPIOx == GPIOB) {
		GPIOB_PCLK_EN();
	}else if(pGPIOx == GPIOC) {
		GPIOC_PCLK_EN();
	}
	else if(pGPIOx == GPIOD) {
		GPIOD_PCLK_EN();
	}
	else if(pGPIOx == GPIOE) {
		GPIOE_PCLK_EN();
    }else if(pGPIOx == GPIOF) {
		GPIOF_PCLK_EN();
	}else if(pGPIOx == GPIOG) {
		GPIOG_PCLK_EN();
	}else if(pGPIOx == GPIOH) {
		GPIOH_PCLK_EN();
	}else if(pGPIOx == GPIOI) {
		GPIOI_PCLK_EN();
	}

}



void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t isEnable)
{

	//printf("RCC address: %p \n",RCC);
	//printf("RCC-> AHB1ENR address: %p \n",&(RCC->AHB1ENR));

	if(isEnable == ENABLE) {

		if(pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
	    }else if(pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		}else if(pGPIOx == GPIOI) {
			GPIOI_PCLK_EN();
		}
	}
	else {

		if(pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		}else if(pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();
		}else if(pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		}else if(pGPIOx == GPIOI) {
			GPIOI_PCLK_DI();
		}
	}

}


void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR  ^= ( 1 << PinNumber);

	printf("GPIO_ToggleOutputPin called \n");
}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {

	uint8_t value = 0;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & (0x00000001));
	return value;

}


uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	return ((uint16_t)(pGPIOx->IDR));
}


void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t EnOrDi) {

	if(EnOrDi == GPIO_PIN_SET){
			//write 1 to the output data register at the bit field corresponding to the pin number
			pGPIOx->ODR |= ( 1 << PinNumber);
	}else
	{			//write 0
		pGPIOx->ODR &= ~( 1 << PinNumber);
	}

}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value) {

	pGPIOx->ODR = Value;

}


void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {

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

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {

	uint8_t iprx = IRQNumber / 4;  // to find register
	uint8_t iprx_section = IRQNumber % 4; // to find register section

	// to find the address we have to add (iprx + 4)
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx)) |= (IRQPriority << shift_amount);

}

void GPIO_IRQHandling(uint8_t PinNumber) {

	if(EXTI->PR & (1 << PinNumber)) {
		EXTI->PR |= (1 << PinNumber);
	}

}






