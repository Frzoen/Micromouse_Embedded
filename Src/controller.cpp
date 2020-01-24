/*
 * controller.cpp
 *
 *  Created on: 19.01.2020
 *      Author: bostr
 */


#include "controller.hpp"
#include <math.h>
extern bool first;
/* Phisical */
#define WHEEL_DIAMETER 0.029 //29.0mm
#define WHEEL_DISPLACEMENT (WHEEL_DIAMETER * M_PI) // mm per rotation 91,106186954104003915416658115106
#define WHEEL_PPR 4096.0	// pulses per rotation
#define DISPLACEMENT_PER_TICK (WHEEL_DISPLACEMENT / WHEEL_PPR) // 0,02224272142434179783091227004763
#define DISPLACEMENT_PER_TICK_IN_METERS DISPLACEMENT_PER_TICK // 0,00002224272142434179783091227004763

#define ROBOT_RADIUS	25.0
#define ROBOT_CIRCLE	(2 * M_PI * ROBOT_RADIUS)  // 157,07963267948966192313216916398

double profilerVR;
double profilerVL;
double profilerSR;
double profilerSL;
uint16_t test_status;
uint16_t test_status2;
double test_actualTimeOfMove;
int test_nrInitMove;
int test_nrUpdateControll;
int32_t eR, eL;
double measLeftV;
double measRightV;
double test_finishMoveTime;
double testOutRightPos, testOutLeftPos, testOutLeftVel, testOutRightVel;

double test_errorL;
double test_errorR;

double test_measLeftPos;
double test_measRightPos;
double test_measRightVel;
double test_measLeftVel;

Controller::Controller(Mouse* _mouse_p) :
      leftPidPosition(2.7, 0, 0, 0.01),
      leftPidVelovity(1.5, 0, 0, 0.001),
      rightPidPosition(2.7, 0, 0, 0.01),
      rightPidVelocity(1.5, 0, 0, 0.001),
      mouse_p(_mouse_p)
{
  moveLinearMouse = 0;
  moveAngularMouse = 0;
  finishMoveTime = 0;

  moveDistanceLeftMotor = 0;
  moveDistanceRightMotor = 0;

  actualTimeOfMove = 0;

  moveMaxVelocity = 0;

  measLeftPos = 0;
  measRightPos = 0;

  outLeftPos = 0;
  outLeftVel = 0;
  outRightPos = 0;
  outRightVel = 0;
}

void Controller::InitMove(double lin, double ang, double vMax)
{
  test_status2 = mouse_p->GetStatus();
  // mouse is during movement, do nothing
  if(mouse_p->GetStatus() != ready)
  {
    return;
  }
  test_nrInitMove++;

  actualTimeOfMove = 0;
  measLeftPos = 0;
  measRightPos = 0;

  const double acc = 0.05;
  moveLinearMouse = lin;
  moveAngularMouse = ang;
  moveMaxVelocity = vMax;
  finishMoveTime = (moveLinearMouse / moveMaxVelocity) + (moveMaxVelocity / acc);
  test_finishMoveTime = finishMoveTime;
  ConvertMouseMoveToMotorMove(moveLinearMouse, moveAngularMouse, finishMoveTime);

  trajectoryLeft.CreateNewTrajectory(moveDistanceLeftMotor,
				     moveMaxVelocity,
				     acc);

  trajectoryRight.CreateNewTrajectory(moveDistanceRightMotor,
				      moveMaxVelocity,
				      acc);

  leftPidPosition.PidReset();
  leftPidVelovity.PidReset();
  rightPidPosition.PidReset();
  rightPidVelocity.PidReset();

  // mouse is starting moving
  mouse_p->SetStatus(moving);
}

void Controller::UpdateControll()
{
  test_status = mouse_p->GetStatus();
  if(mouse_p->GetStatus() != moving)
  {
    return;
  }
  test_nrUpdateControll++;
  test_actualTimeOfMove = actualTimeOfMove;

  // actual velocity of mouse
  eL = mouse_p->GetLeftEncoderTicks();
  eR = mouse_p->GetRightEncoderTicks();
  double leftPosDiff = (double)eL * DISPLACEMENT_PER_TICK_IN_METERS;
  measLeftPos += leftPosDiff;
  // the same as pos / 0.01s
  measLeftVel = leftPosDiff * 100;
  test_measLeftVel = measLeftVel;

  double rightPosDiff = (double)eR * DISPLACEMENT_PER_TICK_IN_METERS;
  measRightPos += rightPosDiff;
  measRightVel = rightPosDiff * 100;
  test_measRightVel = measRightVel;

  trajectoryLeft.CalculateTrajectory(actualTimeOfMove);
  trajectoryRight.CalculateTrajectory(actualTimeOfMove);

  profilerVR = trajectoryRight.GetVRef();
  profilerVL= trajectoryLeft.GetVRef();
  profilerSR= trajectoryRight.GetSRef();
  profilerSL= trajectoryLeft.GetSRef();

  test_errorL = trajectoryLeft.GetSRef() - measLeftPos;
  test_errorR = trajectoryRight.GetSRef() - measRightPos;

  test_measLeftPos = measLeftPos;
  test_measRightPos = measRightPos;

  // update pid position
  if((uint16_t)actualTimeOfMove % 10 == 0)
  {
    outLeftPos = leftPidPosition.PidCalculate(trajectoryLeft.GetSRef() - measLeftPos);
    outRightPos = rightPidPosition.PidCalculate(trajectoryRight.GetSRef() - measRightPos);
  }
  testOutRightPos = outRightPos;
  testOutLeftPos = outLeftPos;

  test_errorL = trajectoryLeft.GetSRef() - measLeftPos;
  test_errorR = trajectoryRight.GetSRef() - measRightPos;

  double inLeftVel =
      outLeftPos +
      trajectoryLeft.GetVRef() -
      measLeftV;
  outLeftVel = leftPidVelovity.PidCalculate(inLeftVel);
  testOutLeftVel = outLeftVel;

  double inRightVel =
      outRightPos +
      trajectoryRight.GetVRef() -
      measRightV;
  outRightVel = rightPidVelocity.PidCalculate(inRightVel);
  testOutRightVel = outRightVel;

  mouse_p->LeftMotorSetSpeed(outLeftVel);
  mouse_p->RightMotorSetSpeed(outRightVel);

  /*
  if((trajectoryLeft.GetSRef() - measLeftPos < 0) &&
      (trajectoryRight.GetSRef() - measRightPos < 0))
  {
    actualTimeOfMove = 0;
    first = true;
    mouse_p->SetStatus(ready);
  }
  */

  if(actualTimeOfMove > finishMoveTime)
  {
    actualTimeOfMove = 0;
    first = true;
    mouse_p->SetStatus(ready);
  }
  actualTimeOfMove += 0.01;
}

void Controller::ConvertMouseMoveToMotorMove(double lin, double ang, double t)
{
  // TODO remove zero assigning
  ang = 0;
  t = 0;
  moveDistanceLeftMotor = lin;
  moveDistanceRightMotor = lin;
}
