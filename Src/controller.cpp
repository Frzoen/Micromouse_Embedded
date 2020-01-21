/*
 * controller.cpp
 *
 *  Created on: 19.01.2020
 *      Author: bostr
 */


#include "controller.hpp"
#include "pid.hpp"
#include "trajectorygenerator.hpp"
#include "mouse.hpp"

Controller::Controller()
{
  moveLinearMouse = 0;
  moveAngularMouse = 0;
  moveTime = 0;

  moveDistanceLeftMotor = 0;
  moveDistanceRightMotor = 0;
}

void Controller::MakeMove(double lin, double ang, double t)
{
  const double acc = 0.5;
  Pid leftPidPosition(1, 0, 0);
  Pid leftPidVelovity(1, 0, 0);
  Pid rightPidPosition(1, 0, 0);
  Pid rightPidVelocity(1, 0, 0);
  TrajectoryGenerator trajectoryLeft(moveDistanceLeftMotor,
				     moveDistanceLeftMotor / t,
				     acc);

  moveLinearMouse = lin;
  moveAngularMouse = ang;
  moveTime = t;
  ConvertMouseMoveToMotorMove(moveLinearMouse, moveAngularMouse, moveTime);

  // TODO insert condition to end move
  double errorLeft = moveDistanceLeftMotor - 1;
  while(1)
  {

  }
}

void Controller::ConvertMouseMoveToMotorMove(double lin, double ang, double t)
{
  // TODO remove zero assigning
  ang = 0;
  t = 0;
  moveDistanceLeftMotor = lin;
  moveDistanceRightMotor = lin;
}
