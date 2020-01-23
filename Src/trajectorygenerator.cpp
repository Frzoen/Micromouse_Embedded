/*
 * trajectorygenerator.cpp
 *
 *  Created on: 18.01.2020
 *      Author: bostr
 */

#include "trajectorygenerator.hpp"
#include <cmath>

TrajectoryGenerator::TrajectoryGenerator()
{
    sRef = 0;
    vRef = 0;
    lastVRef = 0;
    profiler_a = 0;
    profiler_b = 0;
    profiler_c = 0;
    profiler_d = 0;
}

void TrajectoryGenerator::CreateNewTrajectory(double _xFinish, double _vMax, double _accel)
{
  xFinish = _xFinish;
  vMax = _vMax;
  accel = _accel;
  double tMax = xFinish/vMax - vMax/accel;

  if (tMax < 0)
  {
    // trapezoidal profile cannot be created for given acceleration
    // so create triangle profiler
    vMax = std::sqrt(xFinish * accel);
    tMax = 0;
  }

  profiler_a = 0;
  profiler_b = vMax/accel;
  profiler_c = vMax/accel + tMax;
  profiler_d = vMax/accel + tMax + vMax/accel;
}

void TrajectoryGenerator::CalculateTrajectory(uint16_t t)
{
  double y = 0;

  if (t <= profiler_a)
  {
    y = 0;
  }
  else if ((profiler_a <= t) && (t <= profiler_b))
  {
    y = (t - profiler_a) / (profiler_b - profiler_a);
  }
  else if ((profiler_b <= t) && (t <= profiler_c))
  {
    y = 1;
  }
  else if ((profiler_c <= t) && (t <= profiler_d))
  {
      y = (profiler_d - t) / (profiler_d - profiler_c);
  }
  else
  {
      y = 0;
  }

  vRef = y * vMax;

  // integrate velocity (trapezoidal method) to get position
  sRef += ((lastVRef + vRef) / 2) * 0.001;

  lastVRef = vRef;
}
