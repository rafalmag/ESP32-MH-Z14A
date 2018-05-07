#include <Arduino.h>
#include <driver/adc.h>
#include "esp_adc_cal.h"

/**
   simple MH-Z14A CO2 level monitor
   ESP32 
   Based on : https://github.com/coniferconifer/co2sensor
   License: Apache License v2
*/

// References
// http://www.winsen-sensor.com/d/files/infrared-gas-sensor/mh-z14a_co2-manual-v1_01.pdf
// http://www.winsen-sensor.com/d/files/PDF/Infrared%20Gas%20Sensor/NDIR%20CO2%20SENSOR/MH-Z14%20CO2%20V2.4.pdf

// connections:
HardwareSerial Serial2(2); // RX(green) to GPIO 16, TX(blue) to GPIO 17
                           // analog (White) to GPIO 36 aka ADC1_CHANNEL_0
#define PWM_CO2 12         // pwm (yellow) to GPIO 12
#define LED 5              // on board LED for Lolin32

uint8_t readCo2Cmd[] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};

void initSerial2()
{
  uint8_t c;
  while (Serial2.available())
  {
    c = Serial2.read();
  }
}

int getCO2FromSerial2()
{
  int returnnum = 0;
  uint16_t co2 = 0;
  uint8_t readBuffer[9] = {};
  int i;
  uint8_t checksum = 0;
  initSerial2();

  Serial2.write(readCo2Cmd, sizeof(readCo2Cmd));
  Serial2.setTimeout(300); //set 300msec timeout
  returnnum = Serial2.readBytes(readBuffer, sizeof(readBuffer));

  if ((readBuffer[0] == 0xFF) && (readBuffer[1] == 0x86))
  {
    for (i = 1; i < 8; i++)
    {
      checksum += readBuffer[i];
    }
    checksum = 0xff - checksum;
    checksum += 1;
#ifdef DEBUG
    Serial.println(checksum, HEX);
    for (i = 0; i < sizeof(readBuffer); i++)
    {
      Serial.print(readBuffer[i], HEX);
      Serial.print(" ");
    }
#endif
    if (readBuffer[8] != checksum)
    {
      Serial.println("check sum error");
      return (-1); // -1 indicates check sum error
    }
    co2 = (uint16_t)readBuffer[2] << 8;
    co2 += readBuffer[3];
    //    Serial.print("CO2 level(ppm):" );
    // Serial.println(co2);
    return (co2);
  }
  else
  {
    return -1;
  }
}

// for ADC
#define V_REF 1100
esp_adc_cal_characteristics_t characteristics;

int getCO2FromAdc()
{
  //Read analog
  // int readRaw = adc1_get_raw(ADC1_CHANNEL_0);
  // Serial.println(String("raw=") + readRaw);
  //Convert adc_reading to voltage in mV
  // float mVoltRaw = readRaw * (3900 / 2047.0);

  uint32_t mVoltRaw = adc1_to_voltage(ADC1_CHANNEL_0, &characteristics);
  if (mVoltRaw <= 200)
  {
    Serial.print("CO2 error read via adc");
    return -1;
  }
  if (mVoltRaw <= 400)
  {
    Serial.print("CO2 preheating");
    return -2;
  }
  // Serial.println(String("volt=") + mVoltRaw + "mV");

  // 0.4 V = 0ppm
  // 2V = 5000ppm
  float mVoltDiff = mVoltRaw - 400;
  // mVoltDiff - ppm
  // 1600mV - 5000
  return (int)(mVoltDiff * 50.0 / 16.0);
}

int waitUntilHigh(uint8_t pin) {
  int start = micros();
  while (digitalRead(pin) < 0.5)
  {
    delay(1);
  }
  return micros()-start;
}

int waitUntilLow(uint8_t pin) {
  int start = micros();
  while (digitalRead(pin) > 0.5)
  {
    delay(1);
  }
  return micros()-start;
}

int getCO2FromPwm()
{
  waitUntilHigh(PWM_CO2);
  int tH = waitUntilLow(PWM_CO2);
  int tL = waitUntilHigh(PWM_CO2);
  return (int)(5000.0 * (float)(tH - 2) / (float)(tH + tL - 4));
}

void setup()
{
  // init pwm
  pinMode(34, INPUT);

  // init adc
  adc1_config_width(ADC_WIDTH_BIT_11);                        //Range 10 - 0-1023 11 - 0-2047
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11); //ADC_ATTEN_DB_11 = 0-3,6V (or 3.9V??)
  esp_adc_cal_get_characteristics(V_REF, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_11, &characteristics);

  // init serial
  Serial2.begin(9600); // communication for MH-Z14A
  initSerial2();       // read out garbage (i have no idea why but this is a must have for mobile battery operation)

  // others
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  Serial.begin(115200);
#ifdef PREHEATING
  int i;
  for (i = 0; i < 180; i++)
  {
    Serial.print(".");
    delay(1000);
  }
#endif
}

void loop()
{
  Serial.println(String("serial: ") + getCO2FromSerial2());
  Serial.println(String("analog: ") + getCO2FromAdc());
  Serial.println(String("pwm   : ") + getCO2FromPwm());
  Serial.println();
  delay(3000);
  digitalWrite(LED, HIGH);
  delay(2000);
  digitalWrite(LED, LOW);
}
