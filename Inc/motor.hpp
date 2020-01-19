#ifndef MOTOR_H
#define MOTOR_H

#include "tim.h"
#include <stdint.h>

class Motor
{
public:
  Motor(TIM_HandleTypeDef* _htim) :
    htim(_htim)
  {

  }

  Motor() {}

  uint32_t GetTicks() const
  {
    return htim->Instance->CNT;
  }

  void SetTicks(uint32_t cnt) const
  {
    htim->Instance->CNT = cnt;
  }

private:

  // physical connections
  GPIO_TypeDef* gpioSensor;
  uint16_t gpioPinSensor;

  // pwm
  TIM_HandleTypeDef* htim;
};


#endif
