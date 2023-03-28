/*
 * button_interrupt.c
 *
 *  Created on: 21-Feb-2023
 *      Author: Wisal Muhammad
 */


#include<string.h>
#include "stm32f407xx.h"

#define BTN_PRESSED 1
#define BTN_RELEASED 0

void delay(void)
{
	for(uint32_t i = 0 ; i < 900000/ 2 ; i ++);
}


int main(void)
{

	GPIO_Handle_t gpioLed,gpioButton;

	memset(&gpioLed,0,sizeof(gpioLed));
	memset(&gpioButton,0,sizeof(gpioButton));


	gpioLed.pGPIOx = GPIOD;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_Init(&gpioLed);

	gpioButton.pGPIOx = GPIOA;
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	gpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	//gpioButton.GPIO_PinConfig.GPIO_PinOPType = GPIO_NO_PUPD;

	GPIO_Init(&gpioButton);

	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_12, GPIO_PIN_RESET);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRI0);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0,ENABLE);

	while(1);

	return 0;

}

void EXTI0_IRQHandler(void) {
	printf("Interrupt handler called \n");
	GPIO_IRQHandling(GPIO_PIN_NO_0);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}

