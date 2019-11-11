/*****************************************
 *  example 1 : using LPTIMER in polling flag
#include "mbed.h"
#include "kl25_lptmr.h"

Serial pc(USBTX, USBRX);
DigitalOut red(LED_RED);

int main(void)
{
    int time;
    lptmrInit(LPTMR_DIV_2, 100);
    lptmrStart(LPTMR_CLK_1kHz_LPO); 
    while (true)
    {
        while(lptmrOverflow()==0);
          if (red.read() == 0)
                red = 1;
            else
                red = 0;
    }
}
******************************************/
/******************************************
 * Example 2 : lptimer and ISR
#include "mbed.h"
#include "kl25_lptmr.h"
#include "kl25_gpio.h"

volatile bool gFlag = false;

void ISR()
{
    gFlag = true;
    fgpioToggle(FPTD, 4); // D2 out
}

Serial pc(USBTX, USBRX);
DigitalOut d2(PTD4);
DigitalOut red(LED_RED);

int main(void)
{
    int time;
    lptmrInit(LPTMR_DIV_2, 100);
   lptmr_attachInterrupt(ISR);
//  lptmrStart(LPTMR_CLK_8MHz_EXT); //OK
//  lptmrStart(LPTMR_CLK_1kHz_LPO); //OK
//  lptmrStart(LPTMR_CLK_4MHz_INTERNAL);//OK
    lptmrStart(LPTMR_CLK_32kHz_INTERNAL);
    while (true)
    {
        if (gFlag)
        {
            if (red.read() == 0)
                red = 1;
            else
                red = 0;
            printf("flag ");
            gFlag = false;
        }
    }
    return 0;
}
***********************************************************/


#include "kl25_lptmr.h"
#include "clk_freqs.h"

void (*fLPTMR0)(void);

// RAM interrupt handler relocated
void LPTimer_IRQ_Handler(){
    fLPTMR0();
    LPTMR0->CSR |= LPTMR_CSR_TCF_MASK;//clear TOF    
}

//init 16 bits LPTMR0 timer with prescaler LPTMR_DIV_XX and period value
void lptmrInit(LPTMR_Prescaler div, int value){
    SIM->SCGC5 |= SIM_SCGC5_LPTMR_MASK;//enable clock for LPTMR0 on SIM
        /* Reset */
    LPTMR0->CSR = 0;
    LPTMR0->CMR = (value-1)&0xFFFF; // 16 bits counter max value
    LPTMR0->CSR&=~LPTMR_CSR_TCF_MASK;//clear compare flag
    LPTMR0->PSR = LPTMR_PSR_PRESCALE(div); //prescaler timer
}
//becarefull clock selection from SIM is for all Timers LPTMR0,1 and 2...
void lptmrStart(LPTMR_Clk_Sel clk){
    SIM->SOPT1 = ((SIM->SOPT1 & ~SIM_SOPT1_OSC32KSEL_MASK) | SIM_SOPT1_OSC32KSEL(3));
    if((clk>3) || (clk==0)) {//internal clock 
        clkInternalSelection(clk>>2); // change internal clock 32k or 4MHz
    }
    LPTMR0->CSR=LPTMR_CSR_TEN_MASK; // enable LPTIMER
    LPTMR0->PSR |=LPTMR_PSR_PCS(clk&3);// choose clock
    SystemCoreClockUpdate();                      /* Get Core Clock Frequency */
}

//relocate LPTIMER_IRQHandler with NVIC CMIS function
void lptmr_attachInterrupt(void (*userFunc)(void)){
fLPTMR0=userFunc;
NVIC_SetVector(LPTimer_IRQn, (uint32_t)LPTimer_IRQ_Handler);
lptmr_attachLPTimer_IRQHandler ();
}

// use LPTMR0_IRQHandler in main
void lptmr_attachLPTimer_IRQHandler (){
LPTMR0->CSR|=LPTMR_CSR_TIE_MASK;
NVIC_ClearPendingIRQ(LPTimer_IRQn);
NVIC_EnableIRQ(LPTimer_IRQn);
__enable_irq();
}

void lptmr_detachInterrupt(){
LPTMR0->CSR&=~LPTMR_CSR_TIE_MASK;
NVIC_DisableIRQ(LPTimer_IRQn);
}
//return pwm channel associated with pin : need by LPTMR0SetPWM()

