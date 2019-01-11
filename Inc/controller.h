/*
 * controller.h
 *
 *  Created on: 11.01.2019
 *      Author: bartek
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "stdbool.h"


typedef enum
{
	MouseStop,
	MouseForward,
	MouseLeft,
	MouseRight
}eMouseState;

extern eMouseState MouseState;

//******************************************
/*
 * Interface functions
 * they are used for setting drivings
 */
bool setDriveForward(double newGoal);
//******************************************

void InitMouseController();
void MouseController();
void DriveForward();
void DriveLeft90();
void DriveRight90();
void DriveStop();
void PidLeft();
void PidRight();

inline static void setMouseState(eMouseState state)
{
	MouseState = state;
}

inline static eMouseState getMouseState()
{
	return MouseState;
}

#endif /* CONTROLLER_H_ */
