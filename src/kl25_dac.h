#if !defined(DAC_H_)
#define DAC_H_  
#include "mbed.h"
enum DAC_LOW_POWER{
    DAC_LOW_POWER_ON=1,
    DAC_LOW_POWER_OFF=0
};
void dacInit(DAC_LOW_POWER val);
void dacWrite(int val);
#endif