/*
 * Led.c
 *
 *  Created on: Apr 11, 2022
 *      Author: Maryna
 */
#include "led.h"
#include "stdio.h"
#include "gpio.h"
#define LedPort				GPIOD
#define NumnerOfLeds 		4
#define ZeroMarginDefault 	0.1f

//define diodes colors
#define Green 		GPIO_PIN_12
#define Orange		GPIO_PIN_13
#define Red 		GPIO_PIN_14
#define Blue 		GPIO_PIN_15



//private typedefs and variables
static uint16_t leds[] = {Green, Orange, Red, Blue};
static float ZeroMargin = 0.1f;

//Privare functions

//Public functions

//1. Show tilt on leds
void Led_ShowTilt(Led_Tilt_t tilt)
{
	if (tilt.y > ZeroMargin)
	{
		HAL_GPIO_WritePin(LedPort, Green, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(LedPort, Green, GPIO_PIN_RESET);
	}

	if (tilt.y < -ZeroMargin)
	{
		HAL_GPIO_WritePin(LedPort, Red, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(LedPort, Red, GPIO_PIN_RESET);
	}


	if (tilt.x > ZeroMargin)
	{
		HAL_GPIO_WritePin(LedPort, Orange, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(LedPort, Orange, GPIO_PIN_RESET);
	}

	if (tilt.x < -ZeroMargin)
	{
		HAL_GPIO_WritePin(LedPort, Blue, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(LedPort, Blue, GPIO_PIN_RESET);
	}
}

//2. Swich all leds ON
void Led_AllON(void)
{
	uint8_t i;
	for (i = 0; i < NumnerOfLeds; i++)
	{
		HAL_GPIO_WritePin(LedPort, *(leds+i), GPIO_PIN_SET);
	}
}

//3. Switch all leds OFF
void Led_AllOFF(void)
{
	uint8_t i;
	for (i = 0; i < NumnerOfLeds; i++)
	{
		HAL_GPIO_WritePin(LedPort, *(leds+i), GPIO_PIN_RESET);
	}
}

//4. Toggle all leds
void Led_ToggleAll(void)
{
	uint8_t i;
	for (i = 0; i < NumnerOfLeds; i++)
	{
		HAL_GPIO_TogglePin(LedPort, *(leds+i));
	}
}

//5. Set flat position tolerance
void Led_SetZeroMargin(float _ZeroMargin)
{
	ZeroMargin = _ZeroMargin;
}

//6. Init state
void Led_Init()
{
	Led_AllOFF();
	Led_SetZeroMargin(ZeroMarginDefault);
}
