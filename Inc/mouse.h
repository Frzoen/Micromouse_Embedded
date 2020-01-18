/*
 * mouse.h
 *
 *  Created on: 11.01.2019
 *      Author: bartek
 */

#ifndef MOUSE_H_
#define MOUSE_H_

#include "main.h"


typedef enum
{
	MouseStop = 0,		//0
	MouseForward,		//1
	MouseTurn			//2
} eMouseState;

typedef struct
{
	eMouseState MouseState;

	volatile uint16_t VCNL4010ReadingFront;
	volatile uint16_t VCNL4010ReadingFrontLeft;
	volatile uint16_t VCNL4010ReadingFrontRight;
	volatile uint16_t VCNL4010ReadingLeft;
	volatile uint16_t VCNL4010ReadingRight;

} sMouse;

extern sMouse mouse;

#endif /* MOUSE_H_ */
