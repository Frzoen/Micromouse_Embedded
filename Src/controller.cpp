/*
 * controller.cpp
 *
 *  Created on: 19.01.2020
 *      Author: bostr
 */


#include "controller.hpp"
#include <math.h>

/* Phisical */
#define WHEEL_DIAMETER 0.029 //29.0mm
#define WHEEL_DISPLACEMENT (WHEEL_DIAMETER * M_PI) // mm per rotation 91,106186954104003915416658115106
#define WHEEL_PPR 4096.0	// pulses per rotation
#define DISPLACEMENT_PER_TICK (WHEEL_DISPLACEMENT / WHEEL_PPR) // 0,02224272142434179783091227004763
#define DISPLACEMENT_PER_TICK_IN_METERS (DISPLACEMENT_PER_TICK / 1000.0) // 0,00002224272142434179783091227004763

#define ROBOT_RADIUS	25.0
#define ROBOT_CIRCLE	(2 * M_PI * ROBOT_RADIUS)  // 157,07963267948966192313216916398

Controller::Controller(Mouse* _mouse_p) :
  leftPidPosition(1, 0, 0, 0.01),
  leftPidVelovity(1, 0, 0, 0.001),
  rightPidPosition(1, 0, 0, 0.01),
  rightPidVelocity(1, 0, 0, 0.001),
  mouse_p(_mouse_p)
{
  moveLinearMouse = 0;
  moveAngularMouse = 0;
  finishMoveTime = 0;

  moveDistanceLeftMotor = 0;
  moveDistanceRightMotor = 0;

  actualTimeOfMove = 0;

  measLeftPos = 0;
  measRightPos = 0;
}

bool Controller::InitMove(double lin, double ang, double t)
{
  // mouse is during movement, do nothing
  if(mouse_p->GetStatus() != ready)
  {
    return false;
  }

  const double acc = 0.5;
  moveLinearMouse = lin;
  moveAngularMouse = ang;
  finishMoveTime = t;
  actualTimeOfMove = 0;
  measLeftPos = 0;
  measRightPos = 0;
  ConvertMouseMoveToMotorMove(moveLinearMouse, moveAngularMouse, finishMoveTime);

  trajectoryLeft.CreateNewTrajectory(moveDistanceLeftMotor,
				     moveDistanceLeftMotor / finishMoveTime,
				     acc);

  trajectoryRight.CreateNewTrajectory(moveDistanceRightMotor,
				      moveDistanceRightMotor / finishMoveTime,
				      acc);

  leftPidPosition.PidReset();
  leftPidVelovity.PidReset();
  rightPidPosition.PidReset();
  rightPidVelocity.PidReset();

  // mouse is starting moving
  mouse_p->SetStatus(moving);
  return true;
}

void Controller::UpdateControll()
{
  if(mouse_p->GetStatus() != moving)
  {
    return;
  }
  double outLeftPos = 0;
  double outLeftVel = 0;
  double outRightPos = 0;
  double outRightVel = 0;

  // actual velocity of mouse
  uint32_t leftPosDiff = mouse_p->GetLeftEncoderTicks() * DISPLACEMENT_PER_TICK_IN_METERS;
  measLeftPos += leftPosDiff;
  // the same as pos / 0.02s
  double measLeftV = leftPosDiff * 50;

  uint32_t rightPosDiff = mouse_p->GetRightEncoderTicks() * DISPLACEMENT_PER_TICK_IN_METERS;
  measRightPos += rightPosDiff;
  double measRightV = rightPosDiff * 50;

  trajectoryLeft.CalculateTrajectory(actualTimeOfMove);
  trajectoryRight.CalculateTrajectory(actualTimeOfMove);

  // update pid position
  if(actualTimeOfMove % 10 == 0)
  {
    outLeftPos = leftPidPosition.PidCalculate(trajectoryLeft.GetSRef() - measLeftPos);
    outRightPos = rightPidPosition.PidCalculate(trajectoryRight.GetSRef() - measRightPos);
  }

  double inLeftVel =
    outLeftPos +
    trajectoryLeft.GetVRef() -
    measLeftV;
  outLeftVel = leftPidVelovity.PidCalculate(inLeftVel);

  double inRightVel =
    outRightPos +
    trajectoryRight.GetVRef() -
    measRightV;
  outRightVel = rightPidVelocity.PidCalculate(inRightVel);

  mouse_p->LeftMotorSetSpeed(outLeftVel);
  mouse_p->RightMotorSetSpeed(outRightVel);

  if((trajectoryLeft.GetSRef() - measLeftPos <= 0) &&
      (trajectoryRight.GetSRef() - measRightPos <= 0))
  {
    mouse_p->SetStatus(ready);
  }
  actualTimeOfMove++;
}

void Controller::ConvertMouseMoveToMotorMove(double lin, double ang, double t)
{
  // TODO remove zero assigning
  ang = 0;
  t = 0;
  moveDistanceLeftMotor = lin;
  moveDistanceRightMotor = lin;
}
