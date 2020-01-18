/*
 * pid.hpp
 *
 *  Created on: Jan 18, 2020
 *      Author: smroz
 */

#ifndef PID_HPP_
#define PID_HPP_

#define ERR_SUM_MAX		1000

class Pid
{
  float _kp;
  float _ki;
  float _kd;
  float _err;
  float _errSum;
  float _errLast;
public:
  Pid(float kp, float ki, float kd);
  float PidCalculate(float setVal, float readVal);
  uint16_t PidCalculate(uint16_t setVal, uint16_t readVal);
};



#endif /* PID_HPP_ */
