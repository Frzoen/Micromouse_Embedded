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
	MouseStop,
	MouseForward,
	MouseTurn
}eMouseState;

typedef struct
{
	eMouseState MouseState;

	uint16_t VCNL4010Readings[5];

	double desiredPosition;
	double distanceCovered;
	double distanceCoveredLeft;
	double distanceCoveredRight;

	double desiredAngle;
	double angleCovered;
	double circleCoveredLeft;
	double circleCoveredRight;
}sMouse;

extern sMouse mouse;


#endif /* MOUSE_H_ */
