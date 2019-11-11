#if !defined(ADC_H_)
#define ADC_H_  
#include "mbed.h"
void adcInit();

int adcSelectTemp();
int adcSelectBandgapRef();
int adcSelectVref();
uint16_t adcRead(int channel);
int adcSelect(PortName port, int pin);



#endif