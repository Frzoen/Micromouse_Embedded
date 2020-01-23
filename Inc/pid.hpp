/*
 * pid.hpp
 *
 *  Created on: Jan 18, 2020
 *      Author: smroz
 */

#ifndef PID_HPP_
#define PID_HPP_

#include "main.h"

#define ERR_SUM_MAX 1000
#define MAX_OUTPUT 1
#define MIN_OUTPUT (-MAX_OUTPUT)

class Pid
{
  double _kp;
  double _ki;
  double _kd;
  double _dt;
  double _err;
  double _errSum;
  double _errLast;
  double _maxOutput;
  double _minOutput;
  double _errSumMax;
public:
  Pid(double kp, double ki, double kd, double dt);
  double PidCalculate(double error);
  void PidReset();
};



#endif /* PID_HPP_ */
