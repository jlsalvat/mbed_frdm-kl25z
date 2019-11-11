#if !defined(TMP0_H_)
#define TMP0_H_   

#include "PeripheralPins.h"
#include "kl25_util.h"

// functions tpm0
void tpm0Init(TPM_Prescaler div, int value );
void tpm0Start(TPM_Clk_Sel clk);
void tpm0_attachInterrupt(void (*userFunc)(void));
void tpm0_attachTPM0_IRQHandler ();
void tpm0_detachInterrupt();
int tpm0EnablePWM(PinName pin);

// fast function tpm0
void inline  tpm0SetPWM(int channel,int value){
    TPM0->CONTROLS[channel&7].CnV=value&0xFFFF;
}

void inline tpm0SetPeriod(int value){
    TPM0->MOD = (value-1)&0xFFFF; // 16 bits counter max value
}

void inline tpm0Stop(){
    TPM0->SC &= TPM_SC_CMOD(TPM_NO_CLK); // choose clock
}

void inline tpm0Reset(){
    TPM0->CNT=0; // choose clock
}

int inline tpm0Read(){
    return TPM0->CNT;
}

int inline tpm0Overflow(){
    if((TPM0->SC & TPM_SC_TOF_MASK) == 0)
        return 0;
    else{
        TPM0->SC |= TPM_SC_TOF_MASK;/* clear TOF */
        return 1;
    }
}
#endif