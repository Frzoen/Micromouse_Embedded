/*
 * motors.c
 *
 *  Created on: 11.01.2019
 *      Author: bartek
 */


#include "motors.h"

uint16_t test_pwm_left;
uint16_t test_pwm_right;
double test_control_left;
double test_control_right;
void setLeftMotor(double control)
{
	test_control_left = control;
	uint16_t pwm = (uint16_t)(control >= 0 ? control : -control);
	if(pwm > MAX_PWM)
	{
		pwm = MAX_PWM;
	}

	if(control >= 0)
	{
		MOT_LEFT_FORWARD;
		MOT_LEFT_SET_SPEED(pwm);
	}
	else
	{
		MOT_LEFT_BACKWARD;
		MOT_LEFT_SET_SPEED(pwm);
	}
	test_pwm_left = pwm;
}

void setRightMotor(double control)
{
	test_control_right = control;
	uint16_t pwm = (uint16_t)(control >= 0 ? control : -control);
	if(pwm > MAX_PWM)
	{
		pwm = MAX_PWM;
	}

	if(control >= 0)
	{
		MOT_RIGHT_FORWARD;
		MOT_RIGHT_SET_SPEED(pwm);
	}
	else
	{
		MOT_RIGHT_BACKWARD;
		MOT_RIGHT_SET_SPEED(pwm);
	}
	test_pwm_right = pwm;
}
