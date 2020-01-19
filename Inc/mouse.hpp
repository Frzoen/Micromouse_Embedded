#ifndef MOUSE_H
#define MOUSE_H

#include "vcnl4010.hpp"
#include "as5145b.hpp"
#include "TLE9201SGDriver.hpp"
#include "gpio.h"

typedef enum Status{moving = 0, ready} status;

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

	uint32_t GetLeftEncoderTicks() const;
	uint32_t GetRightEncoderTicks() const;

	void SetLeftEncoderTicks(uint32_t cnt) const;
	void SetRightEncoderTicks(uint32_t cnt) const;

	void LeftMotorSetSpeed(double speed) { leftMotor.SetSpeed(speed); }
	void RightMotorSetSpeed(double speed){ rightMotor.SetSpeed(speed); }

	Status GetStatus();
	Status SetStatus(Status _status);


private:

	void DisableAllSensors() const;

	Vcnl4010 leftSensor;
	Vcnl4010 leftMiddleSensor;
	Vcnl4010 frontSensor;
	Vcnl4010 rightMiddleSensor;
	Vcnl4010 rightSensor;

	As5145b leftEncoder;
	As5145b rightEncoder;

	TLE9201SGDriver leftMotor;
	TLE9201SGDriver rightMotor;
	Status status;

};


#endif
