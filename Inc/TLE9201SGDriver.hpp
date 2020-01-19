/*
 * TLE9201SGDriver.hpp
 *
 *  Created on: Jan 18, 2020
 *      Author: smroz
 */

#ifndef TLE9201SGDRIVER_HPP_
#define TLE9201SGDRIVER_HPP_

#include "main.h"
#include "tim.h"
#include "gpio.h"

#define ENABLED 1
#define DISABLED 0
#define REVERSED 1
#define NON_REVERSED 0
#define TIM_CCR1 1
#define TIM_CCR2 2
#define TIM_CCR3 3
#define TIM_CCR4 4


class TLE9201SGDriver
{
  TIM_HandleTypeDef *_htim;
  uint32_t _CCR;
  uint16_t _dirPin;
  GPIO_TypeDef *_dirGPIOx;
  bool  _enable;
  bool _reverse;

public:
  TLE9201SGDriver(TIM_HandleTypeDef *htim, uint32_t CCR, GPIO_TypeDef *dirGPIOx, uint16_t dirPin, bool reverse, bool enable);
  void SetSpeed(double speed);
  void SetDirection(bool direction);
  void EnableDriver(bool enable);

};



#endif /* TLE9201SGDRIVER_HPP_ */
