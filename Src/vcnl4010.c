/*
 * vcnl4010.c
 *
 *  Created on: 10.01.2019
 *      Author: bartek
 */

#include "vcnl4010.h"
#include "i2c.h"

uint8_t test_data_tmp;

bool VCNL4010_begin()
{
	uint8_t valToSend = 0;

	// Set infrared emitter current to 120 mA
	valToSend = 0x0C;
	HAL_I2C_Mem_Write(&hi2c3, VCNL4010_I2CADDR_DEFAULT, VCNL4010_REG_IRLED,
					1, &valToSend, 1, 100);

	// Set proximity measurement rate to 13 measurements/s
	valToSend = 0x03;
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
						1, &valToSend, 1, 10);
	while(1)
	{
		HAL_I2C_Mem_Read(&hi2c3, VCNL4010_I2CADDR_DEFAULT,
								(VCNL4010_REG_COMMAND), 1, &proxAnswer, 1, 10);
		if(proxAnswer & VCNL4010_COMMAND_PROXIMITYREADY)
		{
			HAL_I2C_Mem_Read(&hi2c3, VCNL4010_I2CADDR_DEFAULT,
						(VCNL4010_REG_PROXIMITYDATA_H), 1, measure, 2, 10);
			break;
		}
	}

	result = ((measure[0] << 8) | measure[1]);
	if(result)
	{
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
	}

	return result;
}

uint16_t VCNL4010_readAmbient(void)
{
	return 0;
}


void VCNL4010_Controller()
{

}

