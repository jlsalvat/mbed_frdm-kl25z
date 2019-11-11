
/*
#define IRQ_RISING_EDGE    PORT_PCR_IRQC(9)
#define ENABLE_PIN          PORT_PCR_MUX(1)
#define PULL_UP             PORT_PCR_PE_MASK | PORT_PCR_PS_MASK
#define PIN_D2              (1<<4)
#define PIN_D3              (1<<12)
DigitalOut d3(D3);


void sw2_release(void)
{
    d3 = 1;
    d3 = 0;
   FPTA->PDOR|=(PIN_D3);
   FPTA->PDOR&=~(PIN_D3);
   PORTA->PCR[4]|= PORT_PCR_ISF_MASK; // flag à 0

}*/
/*volatile int x,y;

int main()
{
    while(1)
     y=x+2;*/
    //PIN4
 /*   SIM->SCGC5|=(SIM_SCGC5_PORTB_MASK); // active CLOCK for PORTA
    PORTA->PCR[4] = ENABLE_PIN | IRQ_RISING_EDGE |  PULL_UP; // config PA4
    NVIC_SetVector(PORTA_IRQn,(uint32_t)&sw2_release);
    NVIC_EnableIRQ(PORTA_IRQn);
    while (true) {
    }*/
//}
/*
#include "mbed.h"  
  
#define PTA1    0x02  
#define PTA2    0x04  
#define PTD1    0x01  
#define PTD4    0x10  
#define PTB18    0x40000  
  
static int i = 0;  
  
void PORTD_IRQHandler()  
{  
    PORTB->PTOR |= PTB18;  
    PORTD->ISFR |= PTD4;  
    //NVIC_ClearPendingIRQ(PORTD_IRQn);  
}  
  
void PORTA_IRQHandler()  
{  
    FPTB->PTOR |= PTB18;  
    PORTA->ISFR |= PTA1|PTA2;  
    //NVIC_ClearPendingIRQ(PORTA_IRQn);  
  
}  
int main(void)  
{  
     MCG->C2 |= MCG_C2_IRCS_MASK;        //Fast internal reference clock (4MHz)  
    MCG->SC &= !(MCG_SC_FCRDIV(0x01));    //Diviseur fast internal reference clock /1  
  
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTD_MASK;    //Set up peripheral clock for FPT A,B,D  
    PORTA->PCR[1] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1) | PORT_PCR_IRQC(0x0A);//PTA1 as FPT, Pull Up, interrupt on falling edge  
    PORTA->PCR[2] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1) | PORT_PCR_IRQC(0x0A);//PTA2 as FPT, Pull up, interrupt on falling edge  
    PORTD->PCR[4] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1) | PORT_PCR_IRQC(0x0A);//PTD4 as FPT, Pull up, interrupt on falling edge  
    PORTD->PCR[1] |= PORT_PCR_MUX(1);//PTD1 as FPT  
    PORTB->PCR[18]|= PORT_PCR_MUX(1);//PTB18 as FPT  
  
    //PTA1 & PTA2 as Input  
    FPTA->PDDR &= ~(PTA1 | PTA2);//FPT_PDDR_PDD(1)|FPT_PDDR_PDD(2);      
    FPTD->PDDR &= ~PTD4;//PTD4 as Input  
  
    //Set FPT external Interrupt pin  
    FPTD->PDDR |= PTD1;    //Set PTD1 port direction as input  
    FPTD->PCOR |= PTD1;    //Clear PTD1  
    FPTB->PDDR |= PTB18;    //SetPTB18 port direction as input  
    FPTB->PCOR |= PTB18;    //Clear PTB18  
  
    //Enable External Interupt  
    NVIC_EnableIRQ(PORTA_IRQn);      
    NVIC_EnableIRQ(PORTD_IRQn);      
  
    while(1);  
  
    return 0;  
}  */


/*
int main(void)
{
    SIM->SCGC5 |= 0x400;
    PORTB->PCR[19] = 0x100;
    PTB->PDDR |= 0x80000;
    SIM->SCGC5 |= 0x200;
    PORTA->PCR[1] = 0x103;
    PTA->PDDR &= ~0x02;
    while (1)
    {
        if (PTA->PDIR & 2)
            PTB->PSOR = 0x80000;
        else
            PTB->PCOR = 0x80000;
    }
}*/
/*
int main(){
   // SIM->SCGC5 |= 0x400;
   // PORTB->PCR[19] = 0x100;
   // FPTB->PDDR |= 0x80000;
   gpioEnableCLK(PortA); // D3
    gpioEnableCLK(PortB); // LED RED
    gpioEnableCLK(PortD); // D2
    fgpioInitOut(FPTA,12,0); //D3 out=0
    fgpioInitOut(FPTB,18,1); //LED Red out=0
    fgpioInitIn(FPTD,4); //D2 in
    gpioEnablePin(PORTA,12); 
    gpioEnablePin(PORTB,18); //led RED
    gpioEnablePin(PORTD,4);   
    gpioMode(PORTD,4,pullUp); //D2 pullup
    while(1){
        if(fgpioRead(FPTD,4)==0){
            fgpioSet(FPTA,12);
            fgpioSet(FPTB,18);
        }
        else{
            fgpioReset(FPTA,12);
            fgpioReset(FPTB,18);
        }
        printf("%d\n\r",fgpioRead(FPTD,4));
        fgpioSet(FPTB,18);
        wait(0.1);
        fgpioReset(FPTB,18);
        wait(0.1);
        FPTB->PDOR &= ~0x80000;
        wait(0.1);
        FPTB->PDOR |= 0x80000;
        wait(0.1);       
        
    }
    return 0;
}*/
/*
#include "mbed.h"
#include "kl25_tpm0.h"
#include "kl25_gpio.h"
Serial pc (USBTX,USBRX,115200);
void ISR(){
    fgpioToggle(FPTD,4); //D2 = 5Hz
    fgpioToggle(FPTB,18);//led RED = 5Hz
}
int main(void)
{
    gpioDigitalOut(LED_RED, 1);
    gpioDigitalOut(LED_BLUE, 1);
    gpioDigitalOut(D2, 0);
    //    tpm0Init(TPM_DIV_128, 37500);// 48000000/(35700*128)=0,1s
    tpm0Init(TPM_DIV_1, 48000); // 48000000/(48000)=1ms
                                //   tpm0_attachInterrupt(ISR); // every 0,1s
    tpm0Start(TPM_CLK_PLL_48MHz);
    while (1){
        if (tpm0Overflow()){
            fgpioToggle(FPTD, 4);  //D2 = 5Hz
            fgpioToggle(FPTB, 18); //led RED = 5Hz
            pc.printf("%d\n\r", tpm0Read());
        }
    }
}*/

#include "mbed.h"
#include "kl25_lptmr.h"
#include "kl25_gpio.h"
#include "kl25_sleep.h"
#include "kl25_cmp.h"
#include "kl25_tsi.h"

volatile bool gFlagLPTMR0,gFlagLLW,gFlagInterruptPin,gFlagCMP,gFlagTSI;
Serial pc(USBTX, USBRX);
DigitalIn ptb0(PTB0);
DigitalIn d2(PTD4);
DigitalOut d3(PTA12); 
DigitalOut red(LED_RED);
DigitalOut green(LED_GREEN);
DigitalOut blue(LED_BLUE);

TSIChannelName gChannel;

void ISR_TSI0(){
    gFlagTSI=true;
    fgpioToggle(FPTA, 12); // D3 out   
}
void ISR_LPTMR0(){

    gFlagLPTMR0=true;
    tsiStart(gChannel);
    fgpioToggle(FPTA, 12); // D3 out
}

void ISR_CMP(){

    gFlagCMP=true;
    fgpioToggle(FPTA, 12); // D3 out
}

void ISR_PTD(){

    gFlagInterruptPin=true;
    fgpioToggle(FPTA, 12); // D3 out toggle
    if((PORTD->ISFR & (1<<6))!=0)//PTD6?
        PORTD->ISFR |= 1<<6; // clear interrupt
    if((PORTD->ISFR & (1<<4))!=0)//PTD4?
        PORTD->ISFR |= 1<<4; // clear interrupt
}

volatile int a=0;


int main(void)
{
    sleepInit();
    green=1;
    red=1;
    blue=1;
    ptb0.mode(PullUp);
    //PTD6
    gpioDigitalIn(PTD6);
    gpioEnableCLK(PortD); // D6
    gpioInitIn(FPTD,6); //D6 in
    gpioEnablePin(PORTD,6);//D6  
    gpioMode(PORTD,6,GPIO_MODE_PullUp); //D6 pullup
    gpioPortDEnableInterrupt(6,GPIO_MODE_INTERRUPT_CHANGE);

    d2.mode(PullUp);
    gpioPortDEnableInterrupt(4,GPIO_MODE_INTERRUPT_CHANGE);
    gpioPortD_attachInterrupt(ISR_PTD);

    tsiInit(TSI_CURRENT_8uA, TSI_CURRENT_64uA, TSI_DV_1_03V, TSI_DIV_16, TSI_12_SCAN);
    gChannel=tsiActivateChannel(A0);
    tsiSetTreshold(400,80);
    tsiEnableInterruptOnTreshold();
    tsi_attachInterrupt(ISR_TSI0);

  cmpInit(CMP_hysteresis_5mV);
  cmpSetDacRef(32);
  cmpSetCompare(PTE29,NC);
  cmpEnableInterrupt(CMP_CHANGE);
  cmp_attachInterrupt(ISR_CMP);

 lptmrInit(LPTMR_DIV_8,100);
 lptmrStart(LPTMR_CLK_1kHz_LPO); //OK
 lptmr_attachInterrupt(ISR_LPTMR0);


//  sleepPinEnable(PTB0,SLEEP_WAKUP_CHANGE); Not working on PORTB and PORTC !!!!
     sleepPinEnable(PTD6,SLEEP_WAKUP_CHANGE);
    sleepPinEnable(PTD4,SLEEP_WAKUP_CHANGE);
 sleepSrcEnable(SLEEP_WKUP_FROM_LPTMR);
 sleepSrcEnable(SLEEP_WKUP_FROM_CMP);
 sleepSrcEnable(SLEEP_WKUP_FROM_TSI);
    while (true)
    {
       pc.printf("SLEEP\n\r");
        for(a=0;a<10000;a++);
        sleepLLS();
        pc.printf("RETURN FROM SLEEP\n\r");
        printf("tsiRead()=%d\n\r",tsiRead());
        if(gFlagInterruptPin){
            blue=red=1;
            if (green.read() == 0)
                green = 1;
            else
                green = 0;
            pc.printf("INTERRUPT on PTD6\n\r ");
            gFlagInterruptPin=false;
        }
        if(gFlagLPTMR0){
            green=blue=1;
            if (red.read() == 0)
                red = 1;
            else
                red = 0;
            pc.printf("INTERRUPT from LPTMR0\n\r");
            gFlagLPTMR0=false;
        }
         if(gFlagCMP){
        pc.printf("INTERRUPT from CMP\n\r ");
        red=green=1;
        if (blue.read() == 0)
                blue = 1;
            else
                blue = 0;
        gFlagCMP=false;
        }
         if(gFlagTSI){
        pc.printf("INTERRUPT from TSI\n\r ");
        red=1;
        green=0;
        if (blue.read() == 0)
                blue = 1;
            else
                blue = 0;
        gFlagTSI=false;
        }
     }
    return 0;
}
