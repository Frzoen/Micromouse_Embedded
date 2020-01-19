#include "TLE9201SGDriver.hpp"


TLE9201SGDriver::TLE9201SGDriver(TIM_HandleTypeDef *htim, uint32_t CCR, GPIO_TypeDef *dirGPIOx, uint16_t dirPin , bool reverse, bool enable)
{
  _htim = htim;
  _dirGPIOx = dirGPIOx;
  _dirPin = dirPin;
  _reverse = reverse;
  _enable = enable;
  _CCR = CCR;
}

//speed -1.0 - 1.0;
void TLE9201SGDriver::SetSpeed(double speed)
{
  if (_enable)
  {
    if (speed < 0)
    {
      if(_reverse)
	HAL_GPIO_WritePin(_dirGPIOx,_dirPin,GPIO_PIN_RESET);
      else
	HAL_GPIO_WritePin(_dirGPIOx,_dirPin,GPIO_PIN_SET);

      speed *= -1;
    }
    else
      if(_reverse)
	HAL_GPIO_WritePin(_dirGPIOx,_dirPin,GPIO_PIN_SET);
      else
	HAL_GPIO_WritePin(_dirGPIOx,_dirPin,GPIO_PIN_RESET);

    if (speed > 1.0)
      speed = 1.0;

    if (_CCR == 1)
      _htim->Instance->CCR1 = speed * _htim->Instance->ARR;
    if (_CCR == 2)
      _htim->Instance->CCR2 = speed * _htim->Instance->ARR;
    if (_CCR == 3)
      _htim->Instance->CCR3 = speed * _htim->Instance->ARR;
    if (_CCR == 4)
      _htim->Instance->CCR4 = speed * _htim->Instance->ARR;
  }
  else
  {
    if (_CCR == 1)
      _htim->Instance->CCR1 = 0;
    if (_CCR == 2)
      _htim->Instance->CCR2 = 0;
    if (_CCR == 3)
      _htim->Instance->CCR3 = 0;
    if (_CCR == 4)
      _htim->Instance->CCR4 = 0;
  }


}
void TLE9201SGDriver::EnableDriver(bool enable)
{
  _enable = enable;
}
