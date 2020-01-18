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



void InitMouseController()
{
	setMouseState(MouseStop);
}

void MouseController()
{
	switch (mouse.MouseState)
	{
	case MouseStop:
		break;
	case MouseForward:
		break;
	case MouseTurn:
		break;
	}
}
