#include "kl25_dac.h"
/***************************************
 * Exemple adc conversion 

****************************************/

void dacInit(DAC_LOW_POWER lpOn){
    SIM->SCGC6 |= SIM_SCGC6_DAC0_MASK; // enable CLK ADC0
    DAC0->C0 = DAC_C0_DACEN_MASK | DAC_C0_DACTRGSEL_MASK;// enable and softtrigger
    if(lpOn){
        DAC0->C0 |=DAC_C0_LPEN_MASK;
    }
    DAC0->C1 = 0;

}
void dacWrite(int val){
    DAC0->DAT[0].DATL = val & 0xff;// write low byte
    DAC0->DAT[0].DATH = (val >> 8) & 0x0f; // write high byte
}


