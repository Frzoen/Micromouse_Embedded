/*
 * controller.h
 *
 *  Created on: 11.01.2019
 *      Author: bartek
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "stdbool.h"
#include "mouse.h"


#define MAX_WHEEL_FACTOR 10
//******************************************
/*
 * Interface functions
 * they are used for setting drivings
 */
// newGoal in milimeters
bool setDriveForward(double newGoal);

// angle in degrees: positive - right; negative - left
bool setDriveTurn(double angle);
//******************************************

void InitMouseController();
void MouseController();
void DriveForward();
void DriveTurn();
void DriveStop();
void PidLeft();
void PidRight();
void PidLeftRight(double pathFactor);

inline static void setMouseState(eMouseState state)
{
	mouse.MouseState = state;
}

inline static eMouseState getMouseState()
{
	return mouse.MouseState;
}

#endif /* CONTROLLER_H_ */
