/*
 * vcnl4010.c
 *
 *  Created on: 10.01.2019
 *      Author: bartek
 */

#include "vcnl4010.h"
#include "i2c.h"
#include "mouse.h"
#include "led_switch.h"

/*
 * 0 - VCNL4010_LEFT
 * 1 - VCNL4010_FRONT_LEFT
 * 2 - VCNL4010_FRONT
 * 3 - VCNL4010_FRONT_RIGHT
 * 4 - VCNL4010_RIGHT
 *
 */

bool InitVCNL4010()
{
	uint8_t nrOfDetectedVCNL4010 = 0;
	uint8_t answerVCNL4010 = 0;
	VCNL4010_disableAll();
	HAL_Delay(100);
	LED1_ON;
	HAL_Delay(100);
	VCNL4010_FRONT_ENABLE;
	HAL_I2C_Mem_Read(&hi2c3, VCNL4010_I2CADDR_DEFAULT, (VCNL4010_REG_PRODUCTID),
			1, &answerVCNL4010, 1, 100);
	if (answerVCNL4010 == VCNL4010_ANSWER_ID_REVISION)
	{
		nrOfDetectedVCNL4010++;
		VCNL4010_begin();
	}

	VCNL4010_disableAll();
	HAL_Delay(100);
	LED1_OFF;
	LED2_ON;
	HAL_Delay(100);
	VCNL4010_FRONT_LEFT_ENABLE;
	HAL_I2C_Mem_Read(&hi2c3, VCNL4010_I2CADDR_DEFAULT, (VCNL4010_REG_PRODUCTID),
			1, &answerVCNL4010, 1, 100);
	if (answerVCNL4010 == VCNL4010_ANSWER_ID_REVISION)
	{
		nrOfDetectedVCNL4010++;
		VCNL4010_begin();
	}

	VCNL4010_disableAll();
	HAL_Delay(100);
		LED1_ON;
		LED2_ON;
		HAL_Delay(100);
	VCNL4010_FRONT_RIGHT_ENABLE;
	HAL_I2C_Mem_Read(&hi2c3, VCNL4010_I2CADDR_DEFAULT, (VCNL4010_REG_PRODUCTID),
			1, &answerVCNL4010, 1, 100);
	if (answerVCNL4010 == VCNL4010_ANSWER_ID_REVISION)
	{
		nrOfDetectedVCNL4010++;
		VCNL4010_begin();
	}

	VCNL4010_disableAll();
	HAL_Delay(100);
			LED1_OFF;
			LED2_OFF;
			LED3_ON;
			HAL_Delay(100);
	VCNL4010_LEFT_ENABLE;
	HAL_I2C_Mem_Read(&hi2c3, VCNL4010_I2CADDR_DEFAULT, (VCNL4010_REG_PRODUCTID),
			1, &answerVCNL4010, 1, 100);
	if (answerVCNL4010 == VCNL4010_ANSWER_ID_REVISION)
	{
		nrOfDetectedVCNL4010++;
		VCNL4010_begin();
	}

	VCNL4010_disableAll();
	HAL_Delay(100);
			LED1_ON;
			LED2_OFF;
			LED3_ON;
			HAL_Delay(100);
	VCNL4010_RIGHT_ENABLE;
	HAL_I2C_Mem_Read(&hi2c3, VCNL4010_I2CADDR_DEFAULT, (VCNL4010_REG_PRODUCTID),
			1, &answerVCNL4010, 1, 100);
	if (answerVCNL4010 == VCNL4010_ANSWER_ID_REVISION)
	{
		nrOfDetectedVCNL4010++;
		VCNL4010_begin();
	}
	VCNL4010_disableAll();

	if(nrOfDetectedVCNL4010 == 5)
	{
		return true;
	}
	return false;
}

bool VCNL4010_begin()
{
	uint8_t valToSend = 0;

	// Set infrared emitter current to 120 mA
	valToSend = 20;
	HAL_I2C_Mem_Write(&hi2c3, VCNL4010_I2CADDR_DEFAULT, VCNL4010_REG_IRLED,
					1, &valToSend, 1, 100);

	// Set proximity measurement rate to 31 measurements/s
	valToSend = 0x04;
	HAL_I2C_Mem_Write(&hi2c3, VCNL4010_I2CADDR_DEFAULT, VCNL4010_REG_PROXRATE,
					1, &valToSend, 1, 100);

	return true;
}

uint8_t VCNL4010_getLEDcurrent(void)
{
	uint8_t data_tmp;
	HAL_I2C_Mem_Read(&hi2c3, VCNL4010_I2CADDR_DEFAULT, (VCNL4010_REG_IRLED), 1,
			&data_tmp, 1, 100);
	return data_tmp;
}

void VCNL4010_setLEDcurrent(uint8_t current)
{
	if (current > 20)
		current = 20;
	HAL_I2C_Mem_Write(&hi2c3, VCNL4010_I2CADDR_DEFAULT, VCNL4010_REG_IRLED, 1,
			&current, 1, 100);
}

void VCNL4010_setFrequency(vcnl4010_freq frequency)
{
	HAL_I2C_Mem_Write(&hi2c3, VCNL4010_I2CADDR_DEFAULT, VCNL4010_REG_MODTIMING,
			1, &frequency, 1, 100);
}

uint16_t VCNL4010_readProximity(void)
{
	uint8_t valToSend = VCNL4010_COMMAND_MEASUREPROXIMITY;
	uint8_t proxAnswer = 0;
	uint8_t measure[2] = {0};
	uint16_t result = 0;

	HAL_I2C_Mem_Write(&hi2c3, VCNL4010_I2CADDR_DEFAULT, (VCNL4010_REG_COMMAND),
						1, &valToSend, 1, 2);
	while(1)
	{
		HAL_I2C_Mem_Read(&hi2c3, VCNL4010_I2CADDR_DEFAULT,
								(VCNL4010_REG_COMMAND), 1, &proxAnswer, 1, 2);
		if(proxAnswer & VCNL4010_COMMAND_PROXIMITYREADY)
		{
			HAL_I2C_Mem_Read(&hi2c3, VCNL4010_I2CADDR_DEFAULT,
						(VCNL4010_REG_PROXIMITYDATA_H), 1, measure, 2, 2);
			break;
		}
	}

	result = ((measure[0] << 8) | measure[1]);

	return result;
}

uint16_t VCNL4010_readAmbient(void)
{
	return 0;
}

/*
 * readings performed each 50ms
 */
void VCNL4010_Controller()
{
	// example equation - not used now
	//test_distance = exp(log(68000.0 / VCNL4010_data_16) / 1.765);

	VCNL4010_disableAll();
	VCNL4010_LEFT_ENABLE;
	mouse.VCNL4010ReadingLeft = VCNL4010_readProximity();

	VCNL4010_disableAll();
	VCNL4010_FRONT_LEFT_ENABLE;
	mouse.VCNL4010ReadingFrontLeft= VCNL4010_readProximity();

	VCNL4010_disableAll();
	VCNL4010_FRONT_ENABLE;
	mouse.VCNL4010ReadingFront = VCNL4010_readProximity();

	VCNL4010_disableAll();
	VCNL4010_FRONT_RIGHT_ENABLE;
	mouse.VCNL4010ReadingFrontRight = VCNL4010_readProximity();

	VCNL4010_disableAll();
	VCNL4010_RIGHT_ENABLE;
	mouse.VCNL4010ReadingRight = VCNL4010_readProximity();

	VCNL4010_disableAll();
}

