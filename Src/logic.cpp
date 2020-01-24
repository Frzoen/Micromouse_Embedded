/*
 * logic.cpp
 *
 *  Created on: 19.01.2020
 *      Author: mr_frost
 */

#include "logic.hpp"
bool first= true;
double test_timeToUpdateSensors;

Logic::Logic(Mouse* _mouse_p) :
  mouse_p(_mouse_p),
  controller(_mouse_p)
{
  timeToUpdateSensors = 0;
}

void Logic::UpdateLogic()
{
  RightHandAglorithm();
  controller.UpdateControll();
  // update velocity controller
  test_timeToUpdateSensors = timeToUpdateSensors;
  if(++timeToUpdateSensors == 20) // each 20ms
  {
    timeToUpdateSensors = 0;
    // update sensor measurements
    //mouse_p->UpdateSensorsMeasurements();
  }
}

void Logic::RightHandAglorithm()
{
  // example, go forward
  if(first)
  {
    first = false;
    controller.InitMove(1, 0, 0.3);
  }
}


