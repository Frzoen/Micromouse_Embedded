/*
 * pid.cpp
 *
 *  Created on: Jan 18, 2020
 *      Author: smroz
 */

#include "pid.hpp"

Pid::Pid(double kp, double ki, double kd)
{
  _kp = kp;
  _ki = ki;
  _kd = kd;
  _err = 0;
  _errSum = 0;
  _errLast = 0;
  _maxOutput = MAX_OUTPUT;
  _minOutput = MIN_OUTPUT;
  _errSumMax = ERR_SUM_MAX;
}


double Pid::PidCalculate(double setVal, double readVal)
{
  double errd, u;

  _err = setVal - readVal;
  _errSum += _err;

  if (_errSum > ERR_SUM_MAX) {
    _errSum = ERR_SUM_MAX;
  } else if (_errSum < -ERR_SUM_MAX) {
    _errSum = -ERR_SUM_MAX;
  }

  errd = _err - _errLast;

  u = _kp * _err + _ki * _errSum + _kd * errd;

  return u;
}
