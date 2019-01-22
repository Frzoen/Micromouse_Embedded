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
	MouseStop, MouseForward, MouseTurn
} eMouseState;

typedef struct
{
	eMouseState MouseState;

	volatile uint16_t VCNL4010ReadingFront;
	volatile uint16_t VCNL4010ReadingFrontLeft;
	volatile uint16_t VCNL4010ReadingFrontRight;
	volatile uint16_t VCNL4010ReadingLeft;
	volatile uint16_t VCNL4010ReadingRight;

	volatile double desiredPosition;
	volatile double distanceCovered;
	volatile double distanceCoveredLeft;
	volatile double distanceCoveredRight;

	double desiredAngle;
		double angleCovered;
		double circleCoveredLeft;
		double circleCoveredRight;

	volatile double rightWheelSpeed;
	volatile double leftWheelSpeed;
	volatile double rightWheelDesSpeed;
	volatile double leftWheelDesSpeed;

} sMouse;

extern sMouse mouse;

#endif /* MOUSE_H_ */
