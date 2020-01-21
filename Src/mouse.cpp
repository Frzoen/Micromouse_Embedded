#include "mouse.hpp"

Mouse::Mouse() :
		  leftSensor(DIST5_EN_GPIO_Port, DIST5_EN_Pin, 2000),
		  leftMiddleSensor(DIST4_EN_GPIO_Port, DIST4_EN_Pin, 2000),
		  frontSensor(DIST3_EN_GPIO_Port, DIST3_EN_Pin, 2000),
		  rightMiddleSensor(DIST2_EN_GPIO_Port, DIST2_EN_Pin, 2000),
		  rightSensor(DIST1_EN_GPIO_Port, DIST1_EN_Pin, 2000),
		  leftEncoder(&htim5),
		  rightEncoder(&htim2),
		  leftMotor(&htim3,TIM_CCR1,MOT2_DIR_GPIO_Port,MOT2_DIR_Pin, NON_REVERSED, ENABLED),
		  rightMotor(&htim3,TIM_CCR2,MOT1_DIR_GPIO_Port,MOT1_DIR_Pin, NON_REVERSED, ENABLED)
{
	status = moving;
	InitSensors();
	leftMotor.SetSpeed(0);
	rightMotor.SetSpeed(0);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
	status = ready;
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

Status Mouse::GetStatus()
{
	return status;
}

Status Mouse::SetStatus(Status _status)
{
	status = _status;
	return status;
}
