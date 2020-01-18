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
#include "vcnl4010.h"
#include "main.h"
#include "led_switch.h"

void InitMouseController();

inline static void setMouseState(eMouseState state)
{
	mouse.MouseState = state;
}

inline static eMouseState getMouseState()
{
	return mouse.MouseState;
}

#endif /* CONTROLLER_H_ */
