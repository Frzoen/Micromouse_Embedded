/*
 * pid.hpp
 *
 *  Created on: Jan 18, 2020
 *      Author: smroz
 */

#ifndef PID_HPP_
#define PID_HPP_

#include "main.h"

#define ERR_SUM_MAX		1000

class Pid
{
  double _kp;
  double _ki;
  double _kd;
  double _err;
  double _errSum;
  double _errLast;
public:
  Pid(double kp, double ki, double kd);
  double PidCalculate(double setVal, double readVal);
};



#endif /* PID_HPP_ */