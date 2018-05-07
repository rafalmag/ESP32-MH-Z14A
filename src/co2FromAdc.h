#ifndef _CO2_FROM_ADC_H_
#define _CO2_FROM_ADC_H_

#include <driver/adc.h>
#include "esp_adc_cal.h"

#define V_REF 1100

/**
   ADC1_CHANNEL_0 pin hardcoded, ie. GPIO_36
 */
class Co2FromAdc
{
private:
  esp_adc_cal_characteristics_t characteristics;

public:
  void init();
  int getCO2();
};

#endif /*_CO2_FROM_ADC_H_*/
