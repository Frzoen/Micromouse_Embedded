/*
 * controller.c
 *
 *  Created on: 11.01.2019
 *      Author: bartek
 */


#include "controller.h"
#include "motors.h"
#include <math.h>

eMouseState MouseState;
double desiredPosition;
static double distanceCovered;

double test_error;
double test_distanceCovered;
double distanceCoveredLeft;
double distanceCoveredRight;
double test_left;
double test_right;

void InitMouseController()
{
	setMouseState(MouseStop);
	desiredPosition = 0;
	distanceCovered = 0;
	distanceCoveredLeft = 0;
	distanceCoveredRight = 0;
}

void MouseController()
{
	switch(MouseState)
	{
	case MouseStop:
		DriveStop();
		break;
	case MouseForward:
		DriveForward();
		break;
	case MouseLeft:
		DriveLeft90();
		break;
	case MouseRight:
		DriveRight90();
		break;
	}
}


bool setDriveForward(double newGoal)
{
	// we must wait to end of this task
	if(getMouseState() != MouseStop)
	{
		return false;
	}
	else
	{
		desiredPosition = newGoal;
		distanceCovered = 0;
		setMouseState(MouseForward);
		return true;
	}
}
/*
 * The goal is to reach desired position
 */
void DriveForward()
{
	// average length reached by robot
	int16_t diffLeft = (int16_t)DEFAULT_ENCODER_CNT - (int16_t)LEFT_ENCODER_CNT;
	int16_t diffRight = (int16_t)DEFAULT_ENCODER_CNT - (int16_t)RIGHT_ENCODER_CNT;
	double tmpDistance = ((diffLeft + diffRight) / 2.0) * DISPLACEMENT_PER_TICK;
	// summing it to whole path
	distanceCovered += tmpDistance;
	test_distanceCovered = distanceCovered;
	// compute error
	// if is negative, stop moving

	double error = desiredPosition - distanceCovered;
	test_error = error;
	if(error <= 0)
	{
		setMouseState(MouseStop);
	}
	else
	{
		PidLeft();
		PidRight();
	}
}

void DriveLeft90()
{

}

void DriveRight90()
{

}

void DriveStop()
{
	MOT_LEFT_SET_SPEED(0);
	MOT_RIGHT_SET_SPEED(0);
	desiredPosition = 0;
	distanceCovered = 0;
	distanceCoveredLeft = 0;
	distanceCoveredRight = 0;
}

void PidLeft()
{
	static const double Kp = 4.0;

	int16_t diffLeft = (int16_t)DEFAULT_ENCODER_CNT - (int16_t)LEFT_ENCODER_CNT;
	LEFT_ENCODER_CNT = DEFAULT_ENCODER_CNT;
	double tmpDistance = diffLeft * DISPLACEMENT_PER_TICK;
	// summing it to whole path
	distanceCoveredLeft += tmpDistance;
	//test_left = distanceCoveredLeft
	double error = desiredPosition - distanceCoveredLeft;
	double output = Kp * error;
	setLeftMotor(output);
}

void PidRight()
{
	static const double Kp = 4.0;

	int16_t diffRight = (int16_t)DEFAULT_ENCODER_CNT - (int16_t)RIGHT_ENCODER_CNT;
	RIGHT_ENCODER_CNT = DEFAULT_ENCODER_CNT;
	double tmpDistance = diffRight * DISPLACEMENT_PER_TICK;
	// summing it to whole path
	distanceCoveredRight += tmpDistance;
	//test_left = distanceCoveredLeft
	double error = desiredPosition - distanceCoveredRight;
	double output = Kp * error;
	setRightMotor(output);
}
