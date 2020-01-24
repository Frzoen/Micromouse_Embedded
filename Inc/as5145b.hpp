#ifndef AS5145B_H
#define AS5145B_H

#include "tim.h"
#include <stdint.h>

class As5145b
{
public:
  As5145b(TIM_HandleTypeDef* _htim) :
    htim(_htim)
  {

  }

  int32_t GetTicks() const
  {
    int32_t tics = htim->Instance->CNT;
    htim->Instance->CNT = 32767;
    return  32767 - tics;
  }

  void SetTicks(uint32_t cnt) const
  {
    htim->Instance->CNT = cnt;
  }

private:

  TIM_HandleTypeDef* htim;
};


#endif
