#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "pid.hpp"
#include "trajectorygenerator.hpp"
#include "mouse.hpp"

class Controller
{
public:
  Controller(Mouse* _mouse);

  void InitMove(double lin, double ang, double vMax);

  void UpdateControll();

private:
  void ConvertMouseMoveToMotorMove(double lin, double ang, double t);


  double moveLinearMouse;
  double moveAngularMouse;

  // time to move
  double finishMoveTime;

  // max velocity of movement
  double moveMaxVelocity;

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
  double measLeftVel = 0;
  double measRightVel = 0;

  // double actual time of move
  double actualTimeOfMove;

  // mouse pointer to access set/get methods from mouse
  Mouse* mouse_p;

  double outLeftPos;
  double outLeftVel;
  double outRightPos;
  double outRightVel;

};


#endif
