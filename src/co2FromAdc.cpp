#include "co2FromAdc.h"
#include <Arduino.h>

void Co2FromAdc::init()
{
    adc1_config_width(ADC_WIDTH_BIT_11);                        //Range 10 - 0-1023 11 - 0-2047
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11); //ADC_ATTEN_DB_11 = 0-3,6V (or 3.9V??)
    esp_adc_cal_get_characteristics(V_REF, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_11, &characteristics);
}

int Co2FromAdc::getCO2()
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
    // mVoltDiff - return ppm
    // 1600mV    - 5000ppm
    return (int)(mVoltDiff * 50.0 / 16.0);
}
