#include "mouse.hpp"

Mouse mouse;

Mouse::Mouse() :
  leftSensor(DIST5_EN_GPIO_Port, DIST5_EN_Pin, 2000),
  leftMiddleSensor(DIST4_EN_GPIO_Port, DIST4_EN_Pin, 2000),
  frontSensor(DIST3_EN_GPIO_Port, DIST3_EN_Pin, 2000),
  rightMiddleSensor(DIST2_EN_GPIO_Port, DIST2_EN_Pin, 2000),
  rightSensor(DIST1_EN_GPIO_Port, DIST1_EN_Pin, 2000)
{
  bool isOK = InitSensors();
}

void Mouse::DisableAllSensors() const
{
  leftSensor.Disable();
  leftMiddleSensor.Disable();
  frontSensor.Disable();
  rightMiddleSensor.Disable();
  rightSensor.Disable();
}

bool Mouse::InitSensors()
{
  // all sensors must be valid
  uint8_t nrOfDetectedVCNL4010 = 0;

  DisableAllSensors();
  HAL_Delay(100);
  leftSensor.Enable();
  if(leftSensor.Init())
  {
    nrOfDetectedVCNL4010++;
  }

  DisableAllSensors();
  HAL_Delay(100);
  leftMiddleSensor.Enable();
  if(leftMiddleSensor.Init())
  {
    nrOfDetectedVCNL4010++;
  }

  DisableAllSensors();
  HAL_Delay(100);
  frontSensor.Enable();
  if(frontSensor.Init())
  {
    nrOfDetectedVCNL4010++;
  }

  DisableAllSensors();
  HAL_Delay(100);
  rightMiddleSensor.Enable();
  if(rightMiddleSensor.Init())
  {
    nrOfDetectedVCNL4010++;
  }

  DisableAllSensors();
  HAL_Delay(100);
  rightSensor.Enable();
  if(rightSensor.Init())
  {
    nrOfDetectedVCNL4010++;
  }

  return nrOfDetectedVCNL4010 == 5;
}

bool Mouse::UpdateSensorsMeasurements()
{
  DisableAllSensors();
  leftSensor.Enable();
  leftSensor.UpdateMeasurement();

  DisableAllSensors();
  leftMiddleSensor.Enable();
  leftMiddleSensor.UpdateMeasurement();

  DisableAllSensors();
  frontSensor.Enable();
  frontSensor.UpdateMeasurement();

  DisableAllSensors();
  rightMiddleSensor.Enable();
  rightMiddleSensor.UpdateMeasurement();

  DisableAllSensors();
  rightSensor.Enable();
  rightSensor.UpdateMeasurement();
}

uint16_t Mouse::GetMeasLeftSensor() const
{
  return leftSensor.GetLastMeasurement();
}

uint16_t Mouse::GetMeasLeftMiddleSensor() const
{
  return leftMiddleSensor.GetLastMeasurement();
}

uint16_t Mouse::GetMeasFrontSensor() const
{
  return frontSensor.GetLastMeasurement();
}

uint16_t Mouse::GetMeasRightMiddleSensor() const
{
  return rightMiddleSensor.GetLastMeasurement();
}

uint16_t Mouse::GetMeasRightSensor() const
{
  return rightSensor.GetLastMeasurement();
}
