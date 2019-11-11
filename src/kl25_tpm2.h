#if !defined(TMP2_H_)
#define TMP2_H_   

#include "PeripheralPins.h"
#include "kl25_util.h"

// functions tpm2
void tpm2Init(TPM_Prescaler div, int value );
void tpm2Start(TPM_Clk_Sel clk);
void tpm2_attachInterrupt(void (*userFunc)(void));
void tpm2_attachTPM2_IRQHandler ();
void tpm2_detachInterrupt();
int tpm2EnablePWM(PinName pin);

// fast function tpm2
void inline  tpm2SetPWM(int channel,int value){
    TPM2->CONTROLS[channel&7].CnV=value&0xFFFF;
}

void inline tpm2SetPeriod(int value){
    TPM2->MOD = (value-1)&0xFFFF; // 16 bits counter max value
}

void inline tpm2Stop(){
    TPM2->SC &= TPM_SC_CMOD(TPM_NO_CLK); // choose clock
}

void inline tpm2Reset(){
    TPM2->CNT=0; // choose clock
}

int inline tpm2Read(){
    return TPM2->CNT;
}

int inline tpm2Overflow(){
    if((TPM2->SC & TPM_SC_TOF_MASK) == 0)
        return 2;
    else{
        TPM2->SC |= TPM_SC_TOF_MASK;/* clear TOF */
        return 1;
    }
}
#endif