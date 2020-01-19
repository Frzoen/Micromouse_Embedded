#include "mouse.hpp"

Mouse mouse;

Mouse::Mouse() :
  leftSensor(DIST5_EN_GPIO_Port, DIST5_EN_Pin, 2000),
  leftMiddleSensor(DIST4_EN_GPIO_Port, DIST4_EN_Pin, 2000),
  frontSensor(DIST3_EN_GPIO_Port, DIST3_EN_Pin, 2000),
  rightMiddleSensor(DIST2_EN_GPIO_Port, DIST2_EN_Pin, 2000),
  rightSensor(DIST1_EN_GPIO_Port, DIST1_EN_Pin, 2000),
  leftEncoder(&htim5),
  rightEncoder(&htim2)
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
  bool updateOk = true;

  DisableAllSensors();
  leftSensor.Enable();
  updateOk = leftSensor.UpdateMeasurement();
  if(!updateOk) { return false; }

  DisableAllSensors();
  leftMiddleSensor.Enable();
  updateOk = leftMiddleSensor.UpdateMeasurement();
  if(!updateOk) { return false; }

  DisableAllSensors();
  frontSensor.Enable();
  updateOk = frontSensor.UpdateMeasurement();
  if(!updateOk) { return false; }

  DisableAllSensors();
  rightMiddleSensor.Enable();
  updateOk = rightMiddleSensor.UpdateMeasurement();
  if(!updateOk) { return false; }

  DisableAllSensors();
  rightSensor.Enable();
  updateOk = rightSensor.UpdateMeasurement();
  if(!updateOk) { return false; }

  return true;
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

uint32_t Mouse::GetLeftEncoderTicks() const
{
  return leftEncoder.GetTicks();
}

uint32_t Mouse::GetRightEncoderTicks() const
{
  return rightEncoder.GetTicks();
}

void Mouse::SetLeftEncoderTicks(uint32_t cnt) const
{
  leftEncoder.SetTicks(cnt);
}

void Mouse::SetRightEncoderTicks(uint32_t cnt) const
{
  rightEncoder.SetTicks(cnt);
}