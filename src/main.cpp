#include <Arduino.h>

#include "co2FromSerial.h"
#include "co2FromAdc.h"
#include "co2FromPwm.h"

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// References
// http://www.winsen-sensor.com/d/files/infrared-gas-sensor/mh-z14a_co2-manual-v1_01.pdf
// http://www.winsen-sensor.com/d/files/PDF/Infrared%20Gas%20Sensor/NDIR%20CO2%20SENSOR/MH-Z14%20CO2%20V2.4.pdf

// connections:
HardwareSerial serialCo2(2); // RX(green) to GPIO 16, TX(blue) to GPIO 17
                             // analog (White) to GPIO 36 aka ADC1_CHANNEL_0
#define PWM_CO2 12           // pwm (yellow) to GPIO 12
#define LED 5                // on board LED for Lolin32

Co2FromSerial co2FromSerial(serialCo2);
Co2FromAdc co2FromAdc;
Co2FromPwm co2FromPwm(PWM_CO2);

//set the LCD address to 0x27 for a 20 chars and 4 line display
LiquidCrystal_I2C lcd(0x27, 20, 4);

void setup()
{
  co2FromSerial.init();
  co2FromAdc.init();
  co2FromPwm.init();

  // others
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  Serial.begin(115200);

  lcd.begin(21, 22); // initialize the lcd with SDA 21 and SCL 22 pins
  lcd.backlight();

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
  int serialCO2 = co2FromSerial.getCO2();
  int adcCo2 = co2FromAdc.getCO2();
  int pwmCo2 = co2FromPwm.getCO2();

  Serial.println(String("serial: ") + serialCo2);
  Serial.println(String("analog: ") + adcCo2);
  Serial.println(String("pwm   : ") + pwmCo2);
  Serial.println();

  lcd.setCursor(0, 0);
  lcd.print(String("serial: ") + serialCo2);
  lcd.setCursor(0, 1);
  lcd.print(String("analog: ") + adcCo2);
  lcd.setCursor(0, 2);
  lcd.print(String("pwm   : ") + pwmCo2);

  delay(3000);
  digitalWrite(LED, HIGH);
  delay(2000);
  digitalWrite(LED, LOW);
}
