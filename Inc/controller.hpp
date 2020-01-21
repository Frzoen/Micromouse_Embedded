#ifndef CONTROLLER_H
#define CONTROLLER_H

class Controller
{
public:
  Controller();

  void MakeMove(double lin, double ang, double t);

private:
  void ConvertMouseMoveToMotorMove(double lin, double ang, double t);


  double moveLinearMouse;
  double moveAngularMouse;

  // time to move
  double moveTime;

  // distance to travel in time moveTime
  double moveDistanceLeftMotor;
  double moveDistanceRightMotor;

};


#endif
