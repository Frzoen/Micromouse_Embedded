#ifndef LED_H
#define LED_H

#include "main.h"

class Led
{
public:
  Led(GPIO_TypeDef* _gpioLed, uint16_t _gpioPinLed, bool _isOn) :
    isOn(_isOn),
    gpioLed(_gpioLed),
    gpioPinLed(_gpioPinLed)
  {
    if(isOn)
    {
      On();
    }
  }


  void On()
  {
    HAL_GPIO_WritePin(gpioLed, gpioPinLed, GPIO_PIN_RESET);
    isOn = true;
  }

  void Off()
  {
    HAL_GPIO_WritePin(gpioLed, gpioPinLed, GPIO_PIN_SET);
    isOn = false;
  }

  bool GetIsOn()
  {
    return isOn;
  }

private:

  // true - led is on, false - led is off
  bool isOn;

  // physical connections
  GPIO_TypeDef* gpioLed;
  uint16_t gpioPinLed;
};

extern Led led1;
extern Led led2;
extern Led led3;


#endif
