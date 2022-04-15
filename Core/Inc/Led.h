/*
 * Led.h
 *
 *  Created on: Apr 11, 2022
 *      Author: Maryna
 */
//#include"gpio.h"

#ifndef INC_LED_H_
#define INC_LED_H_

#endif /* INC_LED_H_ */

//public typedefs and virables
typedef struct
{
	float x;
	float y;
}Led_Tilt_t;

//1. Show tilt on leds
void Led_ShowTilt(Led_Tilt_t tilt);

//2. Swich on all leds
void Led_AllON(void);

//3. Switch off all leds
void Led_AllOFF(void);

//4. Toggle all leds
void Led_ToggleAll(void);

//5. Set flat position tolerance
void Led_SetZeroMargin(float _ZeroMargin);

//6. Init state
void Led_Init();
