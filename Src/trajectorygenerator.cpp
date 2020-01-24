/*
 * trajectorygenerator.cpp
 *
 *  Created on: 18.01.2020
 *      Author: bostr
 */

#include "trajectorygenerator.hpp"
#include <cmath>

double test_profa;
double test_profb;
double test_profc;
double test_profd;
double test_y;
double test_t;

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

  test_profa = profiler_a;
  test_profb = profiler_b;
  test_profc = profiler_c;
  test_profd = profiler_d;

  sRef = 0;
  vRef = 0;
  lastVRef = 0;
}

void TrajectoryGenerator::CalculateTrajectory(double t)
{
  double y = 0;

  if (t <= profiler_a)
  {
    y = 0;
  }
  else if ((profiler_a < t) && (t <= profiler_b))
  {
    y = (t - profiler_a) / (profiler_b - profiler_a);
  }
  else if ((profiler_b < t) && (t <= profiler_c))
  {
    y = 1;
  }
  else if ((profiler_c < t) && (t <= profiler_d))
  {
      y = (profiler_d - t) / (profiler_d - profiler_c);
  }
  else
  {
      y = 0;
  }

  vRef = y * vMax;
  test_y = y;
  test_t = t;
  // integrate velocity (trapezoidal method) to get position
  sRef += ((lastVRef + vRef) / 2) * 0.01;

  lastVRef = vRef;
}
