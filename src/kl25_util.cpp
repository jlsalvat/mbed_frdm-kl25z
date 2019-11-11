#include "kl25_util.h"

PortInfo findPortInfo(PinName pin){
    PWMName pwm = (PWMName)pinmap_peripheral(pin, PinMap_PWM);
    PortInfo portInfo;
    MBED_ASSERT(pwm != (PWMName)NC);
    portInfo.port  = (unsigned int)pin >> PORT_SHIFT;
    portInfo.tpm_n = (pwm >> TPM_SHIFT);
    portInfo.ch_n  = (pwm & 0xFF);
    portInfo.pin_n  = (pin&0xFF)>>2;
    for(int i=0; PinMap_PWM[i].pin != NC;i++) {
        if (PinMap_PWM[i].pin==pin) {
            portInfo.alternate = PinMap_PWM[i].function;
        }
    } 
    return portInfo; 
}

void clkInternalSelection(int clk_internal){
    if(clk_internal==1){//select IRC 4MHz clock
        MCG->SC&=~MCG_SC_FCRDIV_MASK; //div=0;
        MCG->C2 |= (MCG_C2_IRCS_MASK);//select fast IRC clock 4MHz
    }else //select IRC 32kHz clock
    {
        MCG->SC&=~MCG_SC_FCRDIV_MASK;
        MCG->C2 &=~(MCG_C2_IRCS_MASK);//select slow clock 32kHz
    }
    SystemCoreClockUpdate(); //update mbed clk definitions
}



void tpmExternalClockSelection(PortName port, int pin){
    //PTA18/PTA19 not usable connected on XTAL 8MHz
    //pTB16/PTB17 not usable connected on TSI touch sensor
    if((port==PortC && pin ==12)||(port==PortE && pin ==29))
    {
        SIM->SOPT4 &= ~SIM_SOPT4_TPM1CLKSEL_MASK;// enable pins of TPM_CLKIN0
        switch(port){
            case PortC :
                SIM->SCGC5|=SIM_SCGC5_PORTC_MASK; // enable clock for input PIN event
                PORTC->PCR[12]&= ~PORT_PCR_MUX_MASK;                 
                PORTC->PCR[12]|=PORT_PCR_MUX(4);// enable pin alternante4
                break;
            case PortE : 
                SIM->SCGC5|=SIM_SCGC5_PORTE_MASK; // enable clock for input PIN event
                PORTE->PCR[29]&= ~PORT_PCR_MUX_MASK;                 
                PORTE->PCR[29]|=PORT_PCR_MUX(4);// enable pin alternante4
                break;
        }    
    }
    if((port==PortC && pin ==13)||(port==PortE && pin ==30))
    {
        SIM->SOPT4 |= SIM_SOPT4_TPM1CLKSEL_MASK;// enable pins of TPM_CLKIN1
        switch(port){
             case PortC : 
                SIM->SCGC5|=SIM_SCGC5_PORTC_MASK; // enable clock for input PIN event
                PORTC->PCR[13]&= ~PORT_PCR_MUX_MASK;                 
                PORTC->PCR[13]|=PORT_PCR_MUX(4);// enable pin alternante4
                break;
            case PortE : 
                SIM->SCGC5|=SIM_SCGC5_PORTE_MASK; // enable clock for input PIN event
                PORTE->PCR[30]&= ~PORT_PCR_MUX_MASK;                 
                PORTE->PCR[30]|=PORT_PCR_MUX(4);// enable pin alternante4
                break;
        }    
    }

}