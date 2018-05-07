
/**
   simple MH-Z14A CO2 level monitor
   ESP32 
   Based on : https://github.com/coniferconifer/co2sensor
   License: Apache License v2
*/

#ifndef _CO2_FROM_SERIAL_H_
#define _CO2_FROM_SERIAL

#include <Arduino.h>

class Co2FromSerial
{
private:
  HardwareSerial serial;
  static const uint8_t readCo2Cmd[];
  void clearSerialBuffer();

public:
  Co2FromSerial(HardwareSerial serial);
  void init();
  int getCO2();
};

#endif /*_CO2_FROM_SERIAL*/
