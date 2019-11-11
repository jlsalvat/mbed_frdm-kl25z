/***************************
 * Exemple 0 : Timer0 wait overflow ...
#include "kl25_util.h"
#include "mbed.h"
#include "kl25_tpm0.h"
#include "kl25_gpio.h"
Serial pc (USBTX,USBRX,115200);

int main(void)
{
    gpioDigitalOut(LED_RED, 1);
    gpioDigitalOut(LED_BLUE, 1);
    gpioDigitalOut(D2, 0);
    tpm0Init(TPM_DIV_1, 48000); // 48000000/(48000)=1ms
    tpm0Start(TPM_CLK_PLL_48MHz);
    while (1){
        if (tpm0Overflow()){
            fgpioToggle(FPTD, 4);  //D2 = 5Hz
            fgpioToggle(FPTB, 18); //led RED = 5Hz
            pc.printf("%d\n\r", tpm0Read());
        }
    }
}
*****************************************************/

/***************************
 * Exemple 1 : gestion interruption TPM0  RED LED clignote pendant quelques secondes...
#include "mbed.h"
#include "kl25_tpm1.h"
#include "kl25_tpm0.h"
#include "kl25_gpio.h"
Serial pc (USBTX,USBRX);
void ISR(){
    static int i;
    i++;
    if(i>10){
        i=0;
        PTB->PTOR = 0x080000; 
    }
}
int main(void)
{
    int cpt;
    SIM->SCGC5 |= 0x400;
    PORTB->PCR[19] = 0x100;
    PTB->PDDR |= 0x080000;
    tpm0Init(TPM_DIV_4, 10000);//100Hz
    tpm0_attachInterrupt(ISR);
    tpm0Start(TPM_CLK_INTERNAL_4MHz);
    while (1)
    {
        printf("%d : %d\n\r", cpt, tpm0Read());
        cpt++;
        if (cpt > 500)
            tpm0Stop();
    }
}
**********************************************/
/**********************************************
 * Exemple 2 : external clock on PTE29 = 100Hz
 * then led green and D2 = 0,5Hz signal
#include "mbed.h"
#include "kl25_tpm1.h"
#include "kl25_tpm0.h"
#include "kl25_gpio.h"
Serial pc (USBTX,USBRX);
void ISR(){
        PTB->PTOR = 0x080000; //toggle green led
        fgpioToggle(FPTD,4);
}
int main(void){
    gpioDigitalOut(D2,1);
    // green led enable
    SIM->SCGC5 |= 0x400;
    PORTB->PCR[19] = 0x100;
    PTB->PDDR |= 0x080000;
    tpm0Init(TPM_DIV_1, 100);// ISR every 100 clk
    tpm0_attachInterrupt(ISR);
    tpm0Start(TPM_CLK_EXT_PIN);
    tpmExternalClockSelection(PortE,29);
    while (1) {
        pc.printf("%d\n\r",  tpm0Read());
    }
}
******************************************************/
/****************************************************
 * Exemple 3 : init generation PWM 480kHz 50% sur broche D4 et 10% sur D9
 * increment on channel1 with ISR
 * decrement on channel2 in main 
#include "mbed.h"
#include "kl25_tpm0.h"
#include "kl25_systick.h"

int channel1,channel2;
void ISR(){
    static int  cpt1;
        tpm0SetPWM(channel1,cpt1+=5);
        if(cpt1>99)
            cpt1=0;
}

 int main(void)
{
    int cpt2=0;
    tpm0Init(TPM_DIV_1, 100);
    tpm0Start(TPM_CLK_PLL_48MHz); 
    channel1=tpm0EnablePWM(D4);//D4 pin
    printf("PD4=channel=%d\n\r",channel1);
    tpm0SetPWM(channel1,50);

    channel2=tpm0EnablePWM(D9);//D9 pin
    printf("PD5=channel=%d\n\r",channel2);
    tpm0SetPWM(channel2,10);
    tpm0_attachInterrupt(ISR);
    while (1)
    {
       if(cpt2<0)
            cpt2=99;
        tpm0SetPWM(channel2,cpt2-=5);
        delay_us(1); // this is not 1us cause of interrupt
    }
}
*******************************************************/
#include "kl25_tpm0.h"
#include "clk_freqs.h"

void (*fTPM0)(void);

// RAM interrupt handler relocated
void TPM0_IRQ_Handler(){
    fTPM0();
    TPM0->SC |= 0x80;//clear TOF    
}

//init 16 bits TPM0 timer with prescaler TPM_DIV_XX and period value
void tpm0Init(TPM_Prescaler div, int value){
    SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;//enable clock for TPM0 on SIM
if (mcgpllfll_frequency() != 0)   //PLL/FLL is selected for TPM input 1
    SIM->SOPT2 |= (1<<SIM_SOPT2_TPMSRC_SHIFT);
        else
    SIM->SOPT2 |= (2<<SIM_SOPT2_TPMSRC_SHIFT);

    OSC0->CR|=OSC_CR_ERCLKEN_MASK| OSC_CR_EREFSTEN_MASK; // ne sert à rien 

    TPM0->SC = 0; // disable timer while configuring 
    TPM0->MOD = (value-1)&0xFFFF; // 16 bits counter max value
    TPM0->SC |= TPM_SC_TOF_MASK;// clear TOF 
    TPM0->SC |= TPM_SC_PS(div); //prescaler timer
}
//becarefull clock selection from SIM is for all Timers TPM0,1 and 2...
void tpm0Start(TPM_Clk_Sel clk){
    int clk_tpm= clk &3;
//Clock Mode Selection
//Selects the TPM counter clock modes.
//00 LPTPM counter is disabled
//01 LPTPM counter increments on every LPTPM counter clock
//10 LPTPM counter increments on rising edge of LPTPM_EXTCLK synchronized to the TPM counter clock
    TPM0->SC |= TPM_SC_CMOD(clk_tpm); // choose clock
    int clk_sim= (clk>>4)&3;
    if(clk_sim>0){
//TPM clock source select for all TIMERS TPM
//Selects the clock source for the TPM counter clock
//00 Clock disabled (default value)
//01 MCGFLLCLK clock or MCGPLLCLK/2
//10 OSCERCLK clock
//11 MCGIRCLK clock
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(clk_sim);// which clock for all TPM ?
    }
    int clk_internal=(clk>>8);
    clkInternalSelection(clk_internal);
//    printf(" clk=%x , clock sim=%x clk_tpm=%x, clk_internal=%d\n\r",clk, clk_sim,clk_tpm,clk_internal);
}

//relocate TPM0_IRQHandler with NVIC CMIS function
void tpm0_attachInterrupt(void (*userFunc)(void)){
fTPM0=userFunc;
NVIC_SetVector(TPM0_IRQn, (uint32_t)TPM0_IRQ_Handler);
tpm0_attachTPM0_IRQHandler ();
}

// use TPM0_IRQHandler in main
void tpm0_attachTPM0_IRQHandler (){
TPM0->SC|=TPM_SC_TOIE_MASK;
NVIC_EnableIRQ(TPM0_IRQn);
__enable_irq();
}

void tpm0_detachInterrupt(){
TPM0->SC&=~TPM_SC_TOIE_MASK;
NVIC_DisableIRQ(TPM0_IRQn);
}
//return pwm channel associated with pin : need by tpm0SetPWM()
//if channel=-1 don't use tpm0SetPWM(). no channel exist for the pin
int tpm0EnablePWM(PinName pin){
    PortInfo portInfo;
    portInfo=findPortInfo(pin);
     pin_function(pin,portInfo.alternate);
//    printf(" port=%d , pin=%x tpm=%x, channel=%d, alternate = %d\n\r",portInfo.port,portInfo.pin_n,portInfo.tpm_n,portInfo.ch_n,portInfo.alternate);
    //PTA3 on SWD DIO pin
    //PTA0 on SWD CLK pin
    if(portInfo.tpm_n==0){
        TPM0->CONTROLS[portInfo.ch_n&7].CnSC=TPM_CnSC_MSB_MASK|TPM_CnSC_ELSB_MASK;
//MSnB:MSnA=10 and ELSnB:ELSnA10 = 10 : mode = Edge-Aligned PWM(non-inverted)
//Set output on reload, clear output on match
        TPM0->CONTROLS[portInfo.ch_n&7].CnV=0; //reset value
        return portInfo.ch_n;
    }
    return -1;
}
