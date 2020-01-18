#ifndef MOUSE_H
#define MOUSE_H

#include "vcnl4010.hpp"

class Mouse
{
public:
  Mouse();

  bool InitSensors();
  bool UpdateSensorsMeasurements();

  uint16_t GetMeasLeftSensor() const;
  uint16_t GetMeasLeftMiddleSensor() const;
  uint16_t GetMeasFrontSensor() const;
  uint16_t GetMeasRightMiddleSensor() const;
  uint16_t GetMeasRightSensor() const;

private:

  void DisableAllSensors() const;

  Vcnl4010 leftSensor;
  Vcnl4010 leftMiddleSensor;
  Vcnl4010 frontSensor;
  Vcnl4010 rightMiddleSensor;
  Vcnl4010 rightSensor;
};

extern Mouse mouse;


#endif
