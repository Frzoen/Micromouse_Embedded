#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "pid.hpp"
#include "trajectorygenerator.hpp"
#include "mouse.hpp"

class Controller
{
public:
  Controller(Mouse* _mouse);

  bool InitMove(double lin, double ang, double t);

  void UpdateControll();

private:
  void ConvertMouseMoveToMotorMove(double lin, double ang, double t);


  double moveLinearMouse;
  double moveAngularMouse;

  // time to move
  double finishMoveTime;

  // distance to travel in time moveTime
  double moveDistanceLeftMotor;
  double moveDistanceRightMotor;

  // PID regulators
  Pid leftPidPosition;
  Pid leftPidVelovity;
  Pid rightPidPosition;
  Pid rightPidVelocity;

  // trajectories for left and right wheel
  TrajectoryGenerator trajectoryLeft;
  TrajectoryGenerator trajectoryRight;

  // actual distance travelled by mouse
  double measLeftPos = 0;
  double measRightPos = 0;

  // double actual time of move
  uint16_t actualTimeOfMove;

  // mouse pointer to access set/get methods from mouse
  Mouse* mouse_p;

};


#endif
