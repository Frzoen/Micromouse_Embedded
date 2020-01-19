#ifndef VCNL4010_H
#define VCNL4010_H

#include "main.h"
#include <cstdint>

#define VCNL4010_I2CADDR_DEFAULT 	(0x13U << 1U)

// commands and constants
#define VCNL4010_REG_COMMAND 		0x80
#define VCNL4010_REG_PRODUCTID 		0x81
#define VCNL4010_REG_PROXRATE 		0x82
#define VCNL4010_REG_IRLED		0x83
#define VCNL4010_REG_AMBIENTPARAMETER 	0x84
#define VCNL4010_REG_AMBIENTDATA_H 	0x85
#define VCNL4010_REG_AMBIENTDATA_L 	0x86
#define VCNL4010_REG_PROXIMITYDATA_H	0x87
#define VCNL4010_REG_PROXIMITYDATA_L	0x88
#define VCNL4010_REG_INTCONTROL 	0x89
#define VCNL4010_REG_PROXINITYADJUST	0x8A
#define VCNL4010_REG_INTSTAT 		0x8E
#define VCNL4010_REG_MODTIMING 		0x8F

#define VCNL4010_COMMAND_MEASUREAMBIENT 	0x10
#define VCNL4010_COMMAND_MEASUREPROXIMITY 	0x08
#define VCNL4010_COMMAND_AMBIENTREADY		0x40
#define VCNL4010_COMMAND_PROXIMITYREADY 	0x20
#define VCNL4010_ANSWER_ID_REVISION 		0x21


class Vcnl4010
{
public:

	// pin to enable, disable sensor
	Vcnl4010(GPIO_TypeDef* gpio, uint16_t pin, uint16_t _treshold);

	uint16_t GetLastMeasurement() const
	{
		return lastMeasurement;
	}

	uint16_t GetTreshold() const
	{
		return treshold;
	}

	void SetTreshold(uint16_t _treshold)
	{
		treshold = _treshold;
	}


	// init sensor
	// true - init ok, false - something went wrong...
	bool Init() const;

	bool UpdateMeasurement();

	void Enable() const;

	void Disable() const;

	// first - bool indicates if communication via i2c was ok
	// second - led current, only valid if first is true
	bool GetLedcurrent(uint8_t& retVal) const;

	bool SetLedcurrent(uint8_t current) const;

	bool isMeasurementAboveTreshold() const
	{
		return lastMeasurement > treshold;
	}

	enum class vcnl4010_freq
	{
		VCNL4010_1_95 = 0,
				VCNL4010_3_90625 = 1,
				VCNL4010_7_8125 = 2,
				VCNL4010_16_625 = 3,
				VCNL4010_31_25 = 4,
				VCNL4010_62_5 = 5,
				VCNL4010_125 = 6,
				VCNL4010_250 = 7,
	};

	bool SetFrequency(vcnl4010_freq frequency) const;

private:

	// set parameters of sensor
	bool SetParameters() const;

	// last measurement
	uint16_t lastMeasurement;

	// treshold above which wall is expected to be
	uint16_t treshold;

	// physical connections
	GPIO_TypeDef* gpioSensor;
	uint16_t gpioPinSensor;
};


#endif
