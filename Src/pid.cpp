/*
 * pid.cpp
 *
 *  Created on: Jan 18, 2020
 *      Author: smroz
 */

#include "pid.hpp"

Pid::Pid(double kp, double ki, double kd, double dt)
{
  _kp = kp;
  _ki = ki;
  _kd = kd;
  _dt = dt;
  _err = 0;
  _errSum = 0;
  _errLast = 0;
  _maxOutput = MAX_OUTPUT;
  _minOutput = MIN_OUTPUT;
  _errSumMax = ERR_SUM_MAX;
}


double Pid::PidCalculate(double error)
{
  double errd, u;

  _err = error;
  _errSum += _err * _dt;

  if (_errSum > ERR_SUM_MAX)
  {
    _errSum = ERR_SUM_MAX;
  }
  else if (_errSum < -ERR_SUM_MAX)
  {
    _errSum = -ERR_SUM_MAX;
  }

  errd = (_err - _errLast) / _dt;

  u = _kp * _err + _ki * _errSum + _kd * errd;

  return u;
}

void Pid::PidReset()
{
  _err = 0;
  _errSum = 0;
  _errLast = 0;
}
