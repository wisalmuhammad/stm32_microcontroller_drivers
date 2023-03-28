/*
 * led_button.c
 *
 *  Created on: 26-Jan-2023
 *      Author: eapple
 */


#include <stdint.h>
#include "stm32f407xx.h"

#define BTN_PRESSED 1
#define BTN_RELEASED 0

void delay(void)
{
	for(uint32_t i = 0 ; i < 50000 / 2 ; i ++);
}


int main(void)
{

	GPIO_Handle_t GpioLed, GpioButton;

	GpioLed.pGPIOx = GPIOD;

	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode   = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed  = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_NO_PUPD;

	GPIO_Init(&GpioLed);

	GpioButton.pGPIOx = GPIOA;

	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode   = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed  = GPIO_SPEED_FAST;
	//GpioButton.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioButton.GPIO_PinConfig.GPIO_PinOPType = GPIO_NO_PUPD;

	GPIO_Init(&GpioButton);

	while(1) {
		if(GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) == BTN_PRESSED) {
			delay();
			GPIO_WriteToOutputPin(GPIOD,GPIO_PIN_NO_12,BTN_PRESSED);
		} else {
			delay();
			GPIO_WriteToOutputPin(GPIOD,GPIO_PIN_NO_12,BTN_RELEASED);
		}
	}

	return 0;
}
