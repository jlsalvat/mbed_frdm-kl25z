#if !defined(ADC_H_)
#define ADC_H_  
#include "mbed.h"

enum ADC_Low_Power {
    ADC_Low_Power_OFF=0,
    ADC_Low_Power_ON=1
};
enum ADC_Size {
    ADC_Size_8bits = 0,
    ADC_Size_12bits = 1,
    ADC_Size_10bits = 2,
    ADC_Size_16bits = 3
};

//00 Default longest sample time; 20 extra ADCK cycles; 24 ADCK cycles total.
//01 12 extra ADCK cycles; 16 ADCK cycles total sample time.
//10 6 extra ADCK cycles; 10 ADCK cycles total sample time.
//11 2 extra ADCK cycles; 6 ADCK cycles total sample time.
enum ADC_Sample_Time {
    ADC_Sample_Time_Minimal=0,
    ADC_Sample_Time_20_Extra_cycles=0x10,
    ADC_Sample_Time_12_Extra_cycles=0x11,
    ADC_Sample_Time_6_Extra_cycles=0x12,
    ADC_Sample_Time_2_Extra_cycles=0x13
};
//00 4 samples averaged.
//01 8 samples averaged.
//10 16 samples averaged.
//11 32 samples averaged.
enum ADC_Avg {
    ADC_No_Avg=0,
    ADC_Avg_4_Samples_Avg=0x10,
    ADC_Avg_8_Samples_Avg=0x11,
    ADC_Avg_16_Samples_Avg=0x12,
    ADC_Avg_32_Samples_Avg=0x13
};

void adcInit(ADC_Low_Power lp,ADC_Size size,ADC_Sample_Time sampleTime,ADC_Avg avg );
void ADCInitMaxSpeed();
int adcSelectTemp();
int adcSelectBandgapRef();
int adcSelectVref();
uint16_t adcRead(int channel);
int adcSelect(PortName port, int pin);



#endif