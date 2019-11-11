
#include "kl25_cmp.h"
/***************************************************************
 * input on PTE29 (sinus) avec test with Vref/2 -> signal carre on D2
 * 
#include "mbed.h"
#include "kl25_cmp.h"
#include "kl25_gpio.h"

volatile bool gFlag=false;

void ISR(){  
    gFlag=true;
    fgpioToggle(FPTD,4); // D2 out  
}

Serial pc(USBTX,USBRX);
DigitalOut d2(PTD4);

 int main(void){
  cmpInit(CMP_hysteresis_5mV);
  cmpSetDacRef(32);
  cmpSetCompare(PTE29,NC);
  cmpEnableInterrupt(CMP_CHANGE);
  cmp_attachInterrupt(ISR);
     while(true){
         if(gFlag){
             printf("flag ");
             gFlag=false;
         }
     }
     return 0;
}
************************************************************/


void (*fCMP0)(void);

// RAM interrupt handler relocated
void CMP0_IRQ_Handler(){
    fCMP0();
    CMP0->SCR |= CMP_SCR_CFF_MASK|CMP_SCR_CFR_MASK;//clear TOF    
}

//Enable CMP and clock of CMP and set Hysteresis for comparator
void cmpInit(CMPHystValue hysteresis ){
    SIM->SCGC4 |=SIM_SCGC4_CMP_MASK; // Enable HSCMP module clock
    CMP0->SCR   = 0x06;  // Disable all interrupts and clear flags (flags are cleared by this write)
    CMP0->CR0=  CMP_CR0_HYSTCTR(hysteresis); //Filter is disabled with histereis
    //Sampling mode is not selected.
    //Windowing mode is not  selected.
    //Trigger mode is disabled.
    //Low-Speed (LS) Comparison mode selected. In this mode, CMP has slower output propagation delay and lower current consumption.
    //Does not invert the comparator output.
    CMP0->CR1 = CMP_CR1_EN_MASK;// enable CMP
}
//set 6 bits Dac internal COMP ref (0 to 63)
void cmpSetDacRef(int value){
    CMP0->DACCR=CMP_DACCR_DACEN_MASK;// enable DAC 6 bits
    CMP0->DACCR|=CMP_DACCR_VOSEL(value);
}
//Set comparator if one pin is NC then compare with DAC ref 6 bits
void cmpSetCompare(PinName pinP, PinName pinN){
    CMPInputName cmpInputP = (CMPInputName)pinmap_peripheral(pinP, PinMap_CMP);
    if (cmpInputP == (CMPInputName)NC)         // When NC, use DAC0
        cmpInputP = (CMPInputName)0x07;
    CMPInputName cmpInputN = (CMPInputName)pinmap_peripheral(pinN, PinMap_CMP);
    if (cmpInputN == (CMPInputName)NC)         // When NC, use DAC0
        cmpInputN = (CMPInputName)0x07;
    if(cmpInputP < 6) pinmap_pinout(pinP, PinMap_CMP); // Map pins
    if(cmpInputN < 6) pinmap_pinout(pinN, PinMap_CMP); // Map pins
    CMP0->MUXCR = CMP_MUXCR_PSEL(cmpInputP)| CMP_MUXCR_MSEL(cmpInputN);
}
// mode = CMP_FALLING, CMP_RISING or CMP_CHANGE
void cmpEnableInterrupt(CmpModeInterrupt mode){
    CMP0->SCR=0;//RAZ
    if(mode&1) //RISING ?
        CMP0->SCR=CMP_SCR_IER_MASK;
    if(mode&2) //FALLING ?
        CMP0->SCR|= CMP_SCR_IEF_MASK;
}

void cmpDisableInterrupt(){
    CMP0->SCR=0;//RAZ ALL
}

void cmp_attachCMP0_IRQHandler(){
    NVIC_EnableIRQ(CMP0_IRQn);
    __enable_irq();
}

void cmp_attachInterrupt(void (*userFunc)(void)){
    fCMP0 = userFunc;
    NVIC_SetVector(CMP0_IRQn, (uint32_t)CMP0_IRQ_Handler);
    NVIC_EnableIRQ(CMP0_IRQn);
    __enable_irq();
}

void cmp_dettachInterrupt(){
    NVIC_DisableIRQ(CMP0_IRQn);
}

/* not used changed by pinmap_pinout of mbed function
void cmpConfPin(PinName pin){
    int port_n  = (unsigned int)pin >> PORT_SHIFT;
    int pin_n  = (pin&0xFF)>>2;
    printf("port = %d pin=%d\n\r",port_n,pin_n);
    PORT_Type *gpio_port = (PORT_Type *)(PORTA_BASE + 0x1000 * port_n); 
    gpio_port->PCR[pin_n] &= ~PORT_PCR_MUX_MASK;
    gpio_port->PCR[pin_n] |= PORT_PCR_MUX(0); // enable pin no alternate
}
*/



