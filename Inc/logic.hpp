/*
 * logic.hpp
 *
 *  Created on: 19.01.2020
 *      Author: mr_frost
 */

#ifndef LOGIC_HPP_
#define LOGIC_HPP_

#include "mouse.hpp"
#include "controller.hpp"

class Logic
{
private:
  Mouse* mouse_p;
  Controller controller;
  void RightHandAglorithm();

  uint8_t timeToUpdateSensors;

public:
	Logic(Mouse* _mouse_p);
	void UpdateLogic();
};



#endif /* LOGIC_HPP_ */
