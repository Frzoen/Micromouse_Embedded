/*
 * controller.c
 *
 *  Created on: 11.01.2019
 *      Author: bartek
 */


#include "controller.h"
#include "motors.h"
#include "mouse.h"
#include <math.h>


double test_pathFactor;
double test_wheelFactor;
double test_diffRightLeft;
double test_left_pwm;
double test_right_pwm;
double test_sumdiff;

#define MY_ABS(A) ((A) >= 0 ? (A) : (-A))

void InitMouseController()
{
	setMouseState(MouseStop);
	mouse.desiredPosition = 0;
	mouse.distanceCovered = 0;
	mouse.distanceCoveredLeft = 0;
	mouse.distanceCoveredRight = 0;
}

void MouseController()
{
	switch(mouse.MouseState)
	{
	case MouseStop:
		DriveStop();
		break;
	case MouseForward:
		DriveForward();
		break;
	case MouseTurn:
		DriveTurn();
		break;
	}
}


bool setDriveForward(double newGoal)
{
	// we must wait to end of current task
	if(getMouseState() != MouseStop)
	{
		return false;
	}
	else
	{
		mouse.desiredPosition = newGoal;
		mouse.distanceCovered = 0;
		mouse.distanceCoveredLeft = 0;
		mouse.distanceCoveredRight = 0;
		setMouseState(MouseForward);
		return true;
	}
}

bool setDriveTurn(double angle)
{
	// we must wait to end of current task
	if(getMouseState() != MouseStop)
	{
		return false;
	}
	else
	{
		mouse.desiredAngle = angle;
		mouse.angleCovered = 0;
		setMouseState(MouseTurn);
		return true;
	}
}

/*
 * The goal of this function is to reach desired position
 */
void DriveForward()
{
	static const double Kp = 1.0;

	// average length reached by robot
	int16_t diffLeft = (int16_t)DEFAULT_ENCODER_CNT - (int16_t)LEFT_ENCODER_CNT;
	int16_t diffRight = (int16_t)DEFAULT_ENCODER_CNT - (int16_t)RIGHT_ENCODER_CNT;
	double tmpDistance = ((diffLeft + diffRight) / 2.0) * DISPLACEMENT_PER_TICK;
	// summing it to whole path
	mouse.distanceCovered += tmpDistance;

	// compute error
	// if is negative, stop moving
	double error = mouse.desiredPosition - mouse.distanceCovered;
	double output = Kp * error;
	if(error <= 0)
	{
		setMouseState(MouseStop);
	}
	else
	{
		PidLeftRight(output);
	}
}

void DriveTurn()
{

}

void DriveStop()
{
	MOT_LEFT_SET_SPEED(0);
	MOT_RIGHT_SET_SPEED(0);
	mouse.desiredPosition = 0;
	mouse.distanceCovered = 0;
	mouse.distanceCoveredLeft = 0;
	mouse.distanceCoveredRight = 0;
	mouse.desiredAngle = 0;
	mouse.angleCovered = 0;
}

void PidLeft()
{
	static const double Kp = 4.0;

	int16_t diffLeft = (int16_t)DEFAULT_ENCODER_CNT - (int16_t)LEFT_ENCODER_CNT;
	LEFT_ENCODER_CNT = DEFAULT_ENCODER_CNT;
	double tmpDistance = diffLeft * DISPLACEMENT_PER_TICK;
	// summing it to whole path
	mouse.distanceCoveredLeft += tmpDistance;
	//test_left = distanceCoveredLeft
	double error = mouse.desiredPosition - mouse.distanceCoveredLeft;
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
	mouse.distanceCoveredRight += tmpDistance;
	//test_left = distanceCoveredLeft
	double error = mouse.desiredPosition - mouse.distanceCoveredRight;
	double output = Kp * error;
	setRightMotor(output);
}

void PidLeftRight(double pathFactor)
{
	static const double Kp = 2.0;
	static const double Ki = 0.02;
	static double sumDiff;

	// average length reached by robot
	int16_t diffLeft = (int16_t)DEFAULT_ENCODER_CNT - (int16_t)LEFT_ENCODER_CNT;
	LEFT_ENCODER_CNT = DEFAULT_ENCODER_CNT;
	int16_t diffRight = (int16_t)DEFAULT_ENCODER_CNT - (int16_t)RIGHT_ENCODER_CNT;
	RIGHT_ENCODER_CNT = DEFAULT_ENCODER_CNT;
	double tmpDistanceLeft = diffLeft * DISPLACEMENT_PER_TICK;
	// summing it to path of left wheel
	mouse.distanceCoveredLeft += tmpDistanceLeft;
	double tmpDistanceRight = diffRight * DISPLACEMENT_PER_TICK;
	// summing it to path of right wheel
	mouse.distanceCoveredRight += tmpDistanceRight;

	int16_t diffRightLeft = diffRight - diffLeft;
	test_diffRightLeft = diffRightLeft;

	sumDiff += diffRightLeft;
	double nastawa = Ki * sumDiff;

	double leftPwm = TARGET_PWM + nastawa;
	if(MY_ABS(leftPwm) > MAX_PWM)
	{
		leftPwm = MAX_PWM;
	}
	double rightPwm = TARGET_PWM - nastawa;
	if(MY_ABS(rightPwm) > MAX_PWM)
	{
		rightPwm = MAX_PWM;
	}
	setLeftMotor(leftPwm);
	setRightMotor(rightPwm);
}
