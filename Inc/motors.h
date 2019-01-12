/*
 * motors.h
 *
 *  Created on: 11.01.2019
 *      Author: bartek
 */

#ifndef MOTORS_H_
#define MOTORS_H_

#include "main.h"
#include <math.h>

#define _INT16_MAX 						65535
#define _INT16_MID 						32767
#define DEFAULT_ENCODER_CNT				_INT16_MID

// pwm [0...100]
#define MAX_PWM 25

/* Encoder */
#define RIGHT_ENCODER_CNT 				(TIM2->CNT)
#define RIGHT_ENCODER_CNT_SET(val)		(TIM2->CNT = (val))
#define LEFT_ENCODER_CNT				(TIM5->CNT)
#define LEFT_ENCODER_CNT_SET(val)		(TIM5->CNT = (val))

/* Phisical */
#define WHEEL_DIAMETER					29.0
#define WHEEL_DISPLACEMENT				(WHEEL_DIAMETER * M_PI) // mm per rotation
#define WHEEL_PPR						4096.0	// pulses per rotation
#define DISPLACEMENT_PER_TICK			(WHEEL_DISPLACEMENT / WHEEL_PPR)

/* Motor */
#define MOT_RIGHT_ENABLE 				HAL_GPIO_WritePin(MOT1_EN_GPIO_Port, MOT1_EN_Pin, GPIO_PIN_RESET)
#define MOT_RIGHT_DISABLE 				HAL_GPIO_WritePin(MOT1_EN_GPIO_Port, MOT1_EN_Pin, GPIO_PIN_SET)
#define MOT_RIGHT_FORWARD 				HAL_GPIO_WritePin(MOT1_DIR_GPIO_Port, MOT1_DIR_Pin, GPIO_PIN_RESET)
#define MOT_RIGHT_BACKWARD 				HAL_GPIO_WritePin(MOT1_DIR_GPIO_Port, MOT1_DIR_Pin, GPIO_PIN_SET)
#define MOT_RIGHT_SET_SPEED(Duty) 		(TIM3->CCR1 = (Duty))

#define MOT_LEFT_ENABLE 				HAL_GPIO_WritePin(MOT2_EN_GPIO_Port, MOT2_EN_Pin, GPIO_PIN_RESET)
#define MOT_LEFT_DISABLE 				HAL_GPIO_WritePin(MOT2_EN_GPIO_Port, MOT2_EN_Pin, GPIO_PIN_SET)
#define MOT_LEFT_FORWARD 				HAL_GPIO_WritePin(MOT2_DIR_GPIO_Port, MOT2_DIR_Pin, GPIO_PIN_RESET)
#define MOT_LEFT_BACKWARD 				HAL_GPIO_WritePin(MOT2_DIR_GPIO_Port, MOT2_DIR_Pin, GPIO_PIN_SET)
#define MOT_LEFT_SET_SPEED(Duty)		(TIM3->CCR2 = (Duty))


void InitMotors();
void setLeftMotor(double control);
void setRightMotor(double control);

#endif /* MOTORS_H_ */
