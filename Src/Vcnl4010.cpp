/*
 * Vcnl4010.cpp
 *
 *  Created on: 18.01.2020
 *      Author: bostr
 */

#include "vcnl4010.hpp"
#include "i2c.h"
#include "led.hpp"


Vcnl4010::Vcnl4010(GPIO_TypeDef* gpio, uint16_t pin, uint16_t _treshold) :
  treshold(_treshold)
{
  gpioSensor = gpio;
  gpioPinSensor = pin;
  lastMeasurement = 0;
}

bool Vcnl4010::Init() const
{
  HAL_StatusTypeDef status;
  uint8_t answerVCNL4010 = 0;

  status = HAL_I2C_Mem_Read(&hi2c3, VCNL4010_I2CADDR_DEFAULT, (VCNL4010_REG_PRODUCTID),
		            1, &answerVCNL4010, 1, 100);
  if (status != HAL_OK)
  {
    return false;
  }

  if (answerVCNL4010 == VCNL4010_ANSWER_ID_REVISION)
  {
    if (SetParameters() == false)
    {
      return false;
    }
  }

  return true;
}

bool Vcnl4010::SetParameters() const
{
  HAL_StatusTypeDef status;
  uint8_t valToSend = 0;

  // Set infrared emitter current to 120 mA
  valToSend = 20;
  status = HAL_I2C_Mem_Write(&hi2c3, VCNL4010_I2CADDR_DEFAULT, VCNL4010_REG_IRLED,
			     1, &valToSend, 1, 100);
  if (status != HAL_OK)
  {
    return false;
  }

  // Set proximity measurement rate to 31 measurements/s
  valToSend = 0x04;
  status = HAL_I2C_Mem_Write(&hi2c3, VCNL4010_I2CADDR_DEFAULT, VCNL4010_REG_PROXRATE,
			     1, &valToSend, 1, 100);

  return status == HAL_OK;
}

bool Vcnl4010::UpdateMeasurement()
{
  HAL_StatusTypeDef status;
  uint8_t valToSend = VCNL4010_COMMAND_MEASUREPROXIMITY;
  uint8_t proxAnswer = 0;
  uint8_t measure[2] = {0};
  uint16_t result = 0;

  status = HAL_I2C_Mem_Write(&hi2c3, VCNL4010_I2CADDR_DEFAULT, (VCNL4010_REG_COMMAND),
			     1, &valToSend, 1, 2);
  if (status != HAL_OK)
  {
    return false;
  }

  while(1)
  {
    status = HAL_I2C_Mem_Read(&hi2c3, VCNL4010_I2CADDR_DEFAULT,
			      (VCNL4010_REG_COMMAND), 1, &proxAnswer, 1, 2);
    if (status != HAL_OK)
    {
      return false;
    }

    if(proxAnswer & VCNL4010_COMMAND_PROXIMITYREADY)
    {
      status = HAL_I2C_Mem_Read(&hi2c3, VCNL4010_I2CADDR_DEFAULT,
		                (VCNL4010_REG_PROXIMITYDATA_H), 1, measure, 2, 2);
      if (status != HAL_OK)
      {
        return false;
      }
      break;
    }
  }

  result = ((measure[0] << 8) | measure[1]);
  lastMeasurement = result;

  return true;
}

void Vcnl4010::Enable() const
{
  HAL_GPIO_WritePin(gpioSensor, gpioPinSensor, GPIO_PIN_RESET);
}

void Vcnl4010::Disable() const
{
  HAL_GPIO_WritePin(gpioSensor, gpioPinSensor, GPIO_PIN_SET);
}

bool Vcnl4010::GetLedcurrent(uint8_t& retVal) const
{
  HAL_StatusTypeDef status;
  uint8_t data_tmp;

  status = HAL_I2C_Mem_Read(&hi2c3, VCNL4010_I2CADDR_DEFAULT, (VCNL4010_REG_IRLED), 1,
		            &data_tmp, 1, 100);
  retVal = data_tmp;

  return status == HAL_OK;
}

bool Vcnl4010::SetLedcurrent(uint8_t current) const
{
  HAL_StatusTypeDef status;
  if (current > 20)
  {
    current = 20;
  }
  status = HAL_I2C_Mem_Write(&hi2c3, VCNL4010_I2CADDR_DEFAULT, VCNL4010_REG_IRLED, 1,
		             &current, 1, 100);

  return status == HAL_OK;
}

bool Vcnl4010::SetFrequency(vcnl4010_freq frequency) const
{
  HAL_StatusTypeDef status;
  uint8_t valToSend = static_cast<uint8_t>(frequency);
  status = HAL_I2C_Mem_Write(&hi2c3, VCNL4010_I2CADDR_DEFAULT, VCNL4010_REG_MODTIMING,
		             1, &valToSend, 1, 100);

  return status == HAL_OK;
}
