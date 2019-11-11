#if !defined(TMP1_H_)
#define TMP1_H_   

#include "PeripheralPins.h"
#include "kl25_util.h"


void tpm1Init(TPM_Prescaler div, int value );
void tpm1Start(TPM_Clk_Sel clk);
void tpm1_attachInterrupt(void (*userFunc)(void));
void tpm1_attachTPM1_IRQHandler ();
void tpm1_detachInterrupt();
int tpm1EnablePWM(PinName pin);


void inline  tpm1SetPWM(int channel,int value){
        TPM1->CONTROLS[channel&7].CnV=value&0xFFFF;
}

void inline tpm1Stop(){
    TPM1->SC &= TPM_SC_CMOD(TPM_NO_CLK); // choose clock
}

void inline tpm1Reset(){
    TPM1->CNT=0; // choose clock
}

int inline tpm1Read(){
    return TPM1->CNT;
}

int inline  tpm1Overflow(){
    if((TPM1->SC & TPM_SC_TOF_MASK) == 0)
        return 0;
    else{
        TPM1->SC |= TPM_SC_TOF_MASK;// Clear TOF
        return 1;
    }
}
#endif