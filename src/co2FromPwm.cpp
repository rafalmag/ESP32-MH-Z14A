#include "co2FromPwm.h"

int Co2FromPwm::waitUntilHigh(uint8_t pin)
{
    int start = micros();
    while (digitalRead(pin) < 0.5)
    {
        delay(1);
    }
    return micros() - start;
}

int Co2FromPwm::waitUntilLow(uint8_t pin)
{
    int start = micros();
    while (digitalRead(pin) > 0.5)
    {
        delay(1);
    }
    return micros() - start;
}

Co2FromPwm::Co2FromPwm(uint8_t pin) : pin(pin)
{
}

void Co2FromPwm::init()
{
    pinMode(pin, INPUT);
}

int Co2FromPwm::getCO2()
{
    //init
    if (digitalRead(pin) < 0.5)
    {
        // it is low, so
        waitUntilHigh(pin);
    }
    else
    {
        // it is high
        waitUntilLow(pin);
        waitUntilHigh(pin);
    }
    //measure high then low
    int tH = waitUntilLow(pin);
    int tL = waitUntilHigh(pin);
    return (int)(5000.0 * (float)(tH - 2) / (float)(tH + tL - 4));
}
