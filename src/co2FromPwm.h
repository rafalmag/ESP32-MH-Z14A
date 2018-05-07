#ifndef _CO2_FROM_PWM_H_
#define _CO2_FROM_PWM_H_

#include <Arduino.h>

class Co2FromPwm
{
  private:
    const uint8_t pin;

    int waitUntilLow(uint8_t pin);
    int waitUntilHigh(uint8_t pin);

  public:
    Co2FromPwm(uint8_t pin);
    void init();
    int getCO2();
};

#endif /*_CO2_FROM_PWM_H_*/
