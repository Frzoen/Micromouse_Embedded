#ifndef TRAJECTORYGENERATOR_H
#define TRAJECTORYGENERATOR_H

#include <stdint.h>

class TrajectoryGenerator
{
public:

  TrajectoryGenerator(double _xFinish, double _vMax, double _accel);

  void SetVMax(double _vMax)
  {
    vMax = _vMax;
  }

  void CalculateTrajectory(uint16_t t);

  double GetSRef() const
  {
    return sRef;
  }

  double GetVRef() const
  {
    return vRef;
  }

private:

  // length of path in milimeters
  double const xFinish;

  // max velocity in milimeters per second
  double vMax;

  // acceleration in milimiters per second square
  const double accel;

  // reference distance
  double sRef;

  // reference velocity
  double vRef;

  /*
   * trapezoidal profiler
   * profiler_a - lower left point
   * profiler_b - upper left point
   * profiler_c - upper right point
   * profiler_d - lower right point
   */
  double profiler_a;
  double profiler_b;
  double profiler_c;
  double profiler_d;

};

#endif
