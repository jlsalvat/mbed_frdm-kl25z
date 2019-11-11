/**********************************************
 * exemple 1 : test 2 timers 5Hz for 2 timers
#include "mbed.h"
#include "kl25_tpm0.h"
#include "kl25_tpm1.h"
#include "kl25_gpio.h"

void ISR0(){
        fgpioToggle(FPTA,12); //D3 = 5Hz
}
void ISR1(){
        fgpioToggle(FPTB,18);//RED LED = 5Hz
        fgpioToggle(FPTD,4); //D2 = 5Hz
}
int main(void)
{
     gpioDigitalOut(LED_RED,1);
    gpioDigitalOut(D2,0);
    gpioDigitalOut(D3,0);
    tpm1Init(TPM_DIV_128, 37500);// 48000000/(35700*128)=0,1s
    tpm1_attachInterrupt(ISR1); // every 0,1s
    tpm1Start(TPM_CLK_PLL_48MHz);
    tpm0Init(TPM_DIV_128, 37500);// 48000000/(35700*128)=0,1s
    tpm0_attachInterrupt(ISR0); // every 0,1s
    tpm0Start(TPM_CLK_PLL_48MHz);
    while (1)
    {
    }
}
***************************************************/
/**************************************************
 * Example 2 : external pin on TPM1 and internal on TPM0
#include "mbed.h"
#include "kl25_tpm0.h"
#include "kl25_tpm1.h"
#include "kl25_gpio.h"

void ISR0(){
        fgpioToggle(FPTA,12); //D3 = 5Hz
}
void ISR1(){
        fgpioToggle(FPTB,18);//RED LED = 5Hz
        fgpioToggle(FPTD,4); //D2 = 5Hz
}
int main(void)
{
     gpioDigitalOut(LED_RED,1);
    gpioDigitalOut(D2,0);
    gpioDigitalOut(D3,0);
    tpm1Init(TPM_DIV_1, 100);// 48000000/(35700*128)=0,1s
    tpm1_attachInterrupt(ISR1); // every 0,1s
    tpmExternalClockSelection(PortE,29);
    tpm1Start(TPM_CLK_EXT_PIN);
    tpm0Init(TPM_DIV_64, 37500);// 48000000/(35700*128)=0,1s
     tpm0_attachInterrupt(ISR0); // every 0,1s
     tpm0Start(TPM_CLK_PLL_48MHz);
    while (1)
    {
        printf("%d\n\r",tpm1Read());
    }
}
************************************************************/
/************************************************************
 * Exemple 3 : 2 timer à fréquence différentes
#include "mbed.h"
#include "kl25_tpm0.h"
#include "kl25_tpm1.h"
#include "kl25_systick.h"

int channel0,channel1;
void ISR(){
    static int  cpt1;
        tpm0SetPWM(channel0,cpt1+=5);
        if(cpt1>99)
            cpt1=0;
}

 int main(void)
{
    int cpt2=0;

    tpm1Init(TPM_DIV_2, 100);
    tpm1Start(TPM_CLK_PLL_48MHz); 
    channel1=tpm1EnablePWM(D3);
    printf("PD3=channel=%d\n\r",channel1);
    if(channel1>=0)
        tpm1SetPWM(channel1,50);
    else
    {
        printf("error channel timer1");
    }   

    tpm0Init(TPM_DIV_1, 100);
    tpm0Start(TPM_CLK_PLL_48MHz); 
    channel0=tpm0EnablePWM(D9);//D9 pin
    printf("PD5=channel=%d\n\r",channel0);
    tpm0SetPWM(channel0,10);
    tpm0_attachInterrupt(ISR);
    while (1)
    {
      if(cpt2<0)
            cpt2=99;
        tpm1SetPWM(channel1,cpt2-=5);
        delay_us(1); // this is not 1us cause of interrupt
    }
}
**********************************************************/

#include "kl25_tpm1.h"
#include "clk_freqs.h"

void (*fTPM1)(void);

void TPM1_IRQ_Handler(){
    fTPM1();
    TPM1->SC |= 0x80;//clear TOF  
}

//init 16 bits TPM1 timer with prescaler TPM_DIV_XX and period value
void tpm1Init(TPM_Prescaler div, int value){
    SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
if (mcgpllfll_frequency() != 0)   //PLL/FLL is selected for TPM input 1
    SIM->SOPT2 |= (1<<SIM_SOPT2_TPMSRC_SHIFT);
        else
    SIM->SOPT2 |= (2<<SIM_SOPT2_TPMSRC_SHIFT);

    OSC0->CR|=OSC_CR_ERCLKEN_MASK| OSC_CR_EREFSTEN_MASK;

    TPM1->SC = 0; // disable timer while configuring 
    TPM1->MOD = (value-1)&0xFFFF; // 16 bits counter max value
    TPM1->SC |= TPM_SC_TOF_MASK;/* clear TOF */
    TPM1->SC |= TPM_SC_PS(div); //prescaler timer
}
void tpm1Start(TPM_Clk_Sel clk){
    int clk_tpm= clk &3;
//Clock Mode Selection
//Selects the TPM counter clock modes.
//00 LPTPM counter is disabled
//01 LPTPM counter increments on every LPTPM counter clock
//10 LPTPM counter increments on rising edge of LPTPM_EXTCLK synchronized to the TPM counter clock
    TPM1->SC |= TPM_SC_CMOD(clk_tpm); // choose clock
    int clk_sim= (clk>>4)&3;

    if(clk_sim>0){
//TPM clock source select
//Selects the clock source for the TPM counter clock
//00 Clock disabled (default value)
//01 MCGFLLCLK clock or MCGPLLCLK/2
//10 OSCERCLK clock
//11 MCGIRCLK clock
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(clk_sim);// which clock for all TPM ?
    }
    int clk_internal=(clk>>8);
    if(clk_internal==1){//select IRC 4MHz clock
        MCG->SC&=~MCG_SC_FCRDIV_MASK; //div=0;
        MCG->C2 |= (MCG_C2_IRCS_MASK);//select fast IRC clock 4MHz
    }else //select IRC 32kHz clock
    {
        MCG->SC&=~MCG_SC_FCRDIV_MASK;
    }
//    SystemCoreClockUpdate();
//    printf(" clk=%x , clock sim=%x clk_tpm=%x, clk_internal=%d\n\r",clk, clk_sim,clk_tpm,clk_internal);
}
//relocate TPM1_IRQHandler with NVIC CMIS function
void tpm1_attachInterrupt(void (*userFunc)(void)){
fTPM1=userFunc;
TPM1->SC|=TPM_SC_TOIE_MASK;
NVIC_SetVector(TPM1_IRQn, (uint32_t)TPM1_IRQ_Handler);
NVIC_EnableIRQ(TPM1_IRQn);
__enable_irq();
}

// use TPM1_IRQHandler in main
void tpm1_attachTPM1_IRQHandler (){
TPM1->SC|=TPM_SC_TOIE_MASK;
NVIC_EnableIRQ(TPM1_IRQn);
__enable_irq();
}

void tpm1_detachInterrupt(){
TPM1->SC&=~TPM_SC_TOIE_MASK;
NVIC_DisableIRQ(TPM1_IRQn);
}
//return pwm channel associated with pin : need by tpm1SetPWM()
//if channel=-1 don't use tpm0SetPWM(). no channel exist for the pin
int tpm1EnablePWM(PinName pin){
    PortInfo portInfo;
    portInfo=findPortInfo(pin);
     pin_function(pin,portInfo.alternate);
//    printf(" port=%d , pin=%x tpm=%x, channel=%d, alternate = %d\n\r",portInfo.port,portInfo.pin_n,portInfo.tpm_n,portInfo.ch_n,portInfo.alternate);
    //PTA3 on SWD DIO pin
    //PTA0 on SWD CLK pin
    if(portInfo.tpm_n==1){
        TPM1->CONTROLS[portInfo.ch_n&7].CnSC=TPM_CnSC_MSB_MASK|TPM_CnSC_ELSB_MASK;
//MSnB:MSnA=10 and ELSnB:ELSnA10 = 10 : mode = Edge-Aligned PWM(non-inverted)
//Set output on reload, clear output on match
        TPM1->CONTROLS[portInfo.ch_n&7].CnV=0; //reset value
        return portInfo.ch_n;
    }
    return -1;
}
