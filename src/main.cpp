
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
/*
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
*/
/*
#include "mbed.h"
#include "kl25_sleep.h"
#include "kl25_systick.h"

void UART0_init(void) {
    MCG->C1|=MCG_C1_IRCLKEN_MASK|MCG_C1_IREFSTEN_MASK;
SIM->SCGC4 |= 0x0400;// enable clock for UART0 
SIM->SOPT2 |= SIM_SOPT2_UART0SRC(3);//internal clock selection
UART0->C2 = 0;// turn off UART0 while changing configurations 
UART0->BDH = 0x00;
UART0->BDL = 26;// 9600 Baud à 4MHz internal clock 4/16/26=9600 
//UART0->BDL = 312;// 9600 Baud à 48MHz internal clock 48000000/16/312=9600 
UART0->C4 = 0x0F; // Over Sampling Ratio 16 
UART0->C1 = 0x00; // 8-bit data 
UART0->C2 = 0x08; // enable transmit 
SIM->SCGC5 |= 0x0200; // enable clock for PORTA 
PORTA->PCR[2] = 0x0200; // make PTA2 UART0_Tx pin 
}
void UART0_send_char(char c){
    while(!(UART0->S1 & 0x80)) {}// wait for transmit buffer empty 
     UART0->D = c;// send a char 
}
void UART0_send_sz(char sz[]){
    for(int i=0;sz[i]!=0;i++)
        UART0_send_char(sz[i]);
}


Serial pc(USBTX,USBRX);
DigitalOut d2(D2);
DigitalOut red(LED_RED);
int main(){
//sleepGoToVLPRunMode();

//UART0_init();

    while (true)
    {
        UART0_send_sz(" test ");
       d2=1;
       red=1;
       delay_ms(1); //1/48 ms
       d2=0;
       red=0;
       delay_ms(1);
    }
    return 0;
}
*/
/*
// test de vitesse 1 : utilisation de interruptin et Ticker 70kHz max
#include "mbed.h"
#include "kl25_gpio.h"
volatile bool gFlag20ms;
volatile int gCpt,gFreq;

InterruptIn d3(D3);//PTA12
Ticker t;
DigitalOut d2(D2);//PTD4
Serial pc(USBTX,USBRX,115200);

void ISR_20ms(){
    gFlag20ms=true;
    gFreq=gCpt;
    gCpt=0;
}
void ISR_counter(){
    gCpt++;
    fgpioToggle(FPTD,4);
}
main(){
    d3.mode(PullUp);
    t.attach(&ISR_20ms,.02);//toutes les 20ms
    d3.rise(&ISR_counter);
    d3.fall(&ISR_counter);
    while (1){
        if(gFlag20ms){//50Hz
            pc.printf("freq=%d\n\r",gFreq*25);    
            gFlag20ms=false;        
        }

    }    
}
// test de vitesse 2 : utilisation de gpio et systick : 270kHz
#include "mbed.h"
#include "kl25_gpio.h"
#include "kl25_systick.h"
volatile bool gFlag20ms;
volatile int gCpt,gFreq;

InterruptIn d3(D3);//PTA12
DigitalOut d2(D2);//PTD4
Serial pc(USBTX,USBRX,115200);
void ISR_20ms(){
    gFlag20ms=true;
    gFreq=gCpt;
    gCpt=0;

}
void ISR_counter(){
    if (gpioIsInterruptPending(PORTA,12)) 
    {
        fgpioToggle(FPTD,4); // D2 out
        gCpt++;
        gpioClearInterrupt(PORTA,12);  
    } 
}

main(){
    systick_attachInterrupt((48000-1)*20,ISR_20ms);
    gpioPortAEnableInterrupt(12,GPIO_MODE_INTERRUPT_CHANGE);
    gpioPortA_attachInterrupt(ISR_counter);
    while (1){
        if(gFlag20ms){//50Hz        
            pc.printf("freq=%d\n\r",gFreq*25);
            gFlag20ms=false;
        }
    }    
}
// test de vitesse 3 : utilisation de tpm, input capture > 10MHz
#include "mbed.h"
#include "kl25_gpio.h"
#include "kl25_tpm1.h"
#include "kl25_systick.h"
volatile bool gFlag20ms;
volatile int gCpt,gFreq;

DigitalOut d2(D2);//PTD4
Serial pc(USBTX,USBRX,115200);
void ISR_20ms(){
    gFlag20ms=true;
    gFreq=gCpt+tpm1Read();
    gCpt=0;
    tpm1Reset();
    fgpioToggle(FPTD,4);//PTD4
}
void ISR_OVF_TIMER(){
    gCpt+=0xFFFF;
}

main(){
    systick_attachInterrupt((48000-1)*20,ISR_20ms);
    tpm1Init(TPM_DIV_1, 0xFFFF);// 48000000/(65536)=7,3kHz
    tpm1_attachInterrupt(ISR_OVF_TIMER); // every overflow
    tpmExternalClockSelection(PortE,29);//input pin
    tpm1Start(TPM_CLK_EXT_PIN);
    while (1){
        if(gFlag20ms){//50Hz
            pc.printf("freq=%d\n\r",gFreq*50);
            gFlag20ms=false;
        }
    }    
}


//génération rampe CNA directe AnalogOut : entre chaque echantillons = 1,8us
#include "mbed.h"
#include "kl25_gpio.h"

AnalogOut a0(PTE30);
DigitalOut d2(D2);//PTD4

main(){
    unsigned short cpt;
    while (1){
        cpt+=32; //de 2 en 2
        a0.write_u16(cpt);
        fgpioToggle(FPTD,4);//max speed
    }    
}

//génération rampe CNA directe dac function :  entre chaque echantillons = 0,8us
#include "mbed.h"
#include "kl25_dac.h"
#include "kl25_gpio.h"

DigitalOut d2(D2);//PTD4

main(){
    unsigned short cpt;
    dacInit(DAC_LOW_POWER_OFF);
    while (1){
        cpt+=2;
        if(cpt>4095)
            cpt=0;
        dacWrite(cpt);
        fgpioToggle(FPTD,4);//max speed
    }    
}
//lecture analogique : temps de conversion : 35us

#include "mbed.h"
#include "kl25_gpio.h"

Serial pc(USBTX,USBRX,115200);
AnalogIn a0(A0);
DigitalOut d2(D2);//PTD4

main(){
    unsigned short val;
    while (1){
        val=a0.read_u16();
        pc.printf("%d\n\r",val);
        fgpioToggle(FPTD,4);//max speed
    }    
}

//idem mais plus vite avec modif pour aller à fond... : 2,9us
// on peut même faire mieux... Cf prog IIR
#include "mbed.h"
#include "kl25_adc.h"
#include "kl25_gpio.h"

Serial pc(USBTX,USBRX,115200);
DigitalOut d2(D2);//PTD4

main(){
    unsigned short val;
    adcInit(ADC_Low_Power_OFF,ADC_Size_10bits,ADC_Sample_Time_Minimal,ADC_No_Avg);// vitesse max
    int channel=adcSelect(PortB,0);
    while (1){
        val=adcRead(channel);
        fgpioToggle(FPTD,4);//max speed
      //  pc.printf("%d\n\r",val);
    }    
}*/

// mettre en place un timer rapide pour ADC vers DAC (recopie seulement)
// Puis génération d'un sinux
//puis filtrage rapide.
/*
#include "mbed.h"
#include "kl25_adc.h"
 int main(){
     // enable clock for ports
     uint32_t val_A0,val_Temp,val_band,val_Vref;
     int channel;
    adcInit(ADC_Low_Power_OFF,ADC_Size_10bits,ADC_Sample_Time_6_Extra_cycles,ADC_Avg_4_Samples_Avg);
    while(1){
     val_A0= adcRead(adcSelect(PortB,0));
    val_Temp= adcRead(adcSelectTemp());
    val_Vref= adcRead(adcSelectVref());
    printf("%d %d %d\n\r",val_A0,val_Temp,val_Vref);
    wait_ms(100);
    }
}
*/
/*
// mettre en place un timer rapide pour ADC vers DAC (recopie seulement)
// Puis génération d'un sinux
//puis filtrage 10kHz fe=100000kHz

#include "mbed.h"
#include "kl25_adc.h"
#include "kl25_dac.h"
#include "kl25_gpio.h"

Serial pc(USBTX,USBRX,115200);
DigitalOut d2(D2);//PTD4

//lecture sur PTB0 (A0) analogique -> recopie sur PTE30
// temps de conversion : 3,3us -> CAN -> CNA
// input : 20kHz
//
//FIR filter designed with
// http://t-filter.appspot.com

//sampling frequency: 100000 Hz

//fixed point precision: 16 bits

// 0 Hz - 10000 Hz
//  gain = 1
//  desired ripple = 5 dB
//  actual ripple = n/a

//20000 Hz - 50000 Hz
//  gain = 0
//  desired attenuation = -40 dB
//  actual attenuation = n/a

#define SAMPLEFILTER_TAP_NUM 15

typedef struct {
  int16_t history[SAMPLEFILTER_TAP_NUM];
  uint16_t last_index;
} SampleFilter;

static int16_t filter_taps[SAMPLEFILTER_TAP_NUM] = {
  -413,
  -886,
  -1021,
  -110,
  2180,
  5360,
  8183,
  9315,
  8183,
  5360,
  2180,
  -110,
  -1021,
  -886,
  -413
};

void SampleFilter_init(SampleFilter* f) {
  int i;
  for(i = 0; i < SAMPLEFILTER_TAP_NUM; ++i)
    f->history[i] = 0;
  f->last_index = 0;
}

void SampleFilter_put(SampleFilter* f, int16_t input) {
  f->history[f->last_index++] = input;
  if(f->last_index == SAMPLEFILTER_TAP_NUM)
    f->last_index = 0;
}

int SampleFilter_get(SampleFilter* f) {
  int acc = 0;
  int index = f->last_index, i;
  for(i = 0; i < SAMPLEFILTER_TAP_NUM; ++i) {
    index = index != 0 ? index-1 : SAMPLEFILTER_TAP_NUM-1;
    acc += f->history[index] * filter_taps[i];
  };
  acc>>=16;
  if(acc<0)
    acc=0;
  return acc;
}


SampleFilter gFilter;
main(){
    
    int16_t val,val_filtered;
    adcInit(ADC_Low_Power_OFF,ADC_Size_12bits,ADC_Sample_Time_Minimal,ADC_No_Avg);
    int channel=adcSelect(PortB,0);
    dacInit(DAC_LOW_POWER_OFF);
    SampleFilter_init(&gFilter);
    while (1){
        val=adcRead(channel);
        SampleFilter_put(&gFilter,val);
        val_filtered=SampleFilter_get(&gFilter);
        dacWrite(val_filtered);
        fgpioToggle(FPTD,4); // visu temps de conversion
    }    
}
*/
// IIR Filter Design fs=200kHz
// A commercial license for MicroModeler DSP can be obtained at http://www.micromodeler.com/launch.jsp

// Begin header file, filter1.h
#include "mbed.h"
#include "kl25_adc.h"
#include "kl25_dac.h"
#include "kl25_gpio.h"

Serial pc(USBTX,USBRX,115200);
DigitalOut d2(D2);//PTD4

//float f_filter_coefficients[]={
//    0.07718949372345962, 0.15437898744691925, 0.07718949372345962, 1.0485995763626117, -0.2961403575616696,// b0, b1, b2, a1, a2
//    0.0625, 0.125, 0.0625, 1.3209134308194261, -0.6327387928852763// b0, b1, b2, a1, a2
//};// butterwoth ordre 4 IIR

float f_filter_coefficients[10] = 
{
// Scaled for floating point

    0.26150331989552333, -0.18260091393549116, 0.26150331989552333, 1.301067063220938, -0.5286084866020655,// b0, 0, b1, b2, a1, a2
    0.5, -0.7551046162946281, 0.5000000000000001, 1.5385144640061934, -0.9242487913974602// b0, 0, b1, b2, a1, a2
};//Elliptic filter fs=0.1 ordre 4 from http://www.micromodeler.com/launch.jsp



#define N 10

int16_t filter1_coefficients[5];
int16_t filter2_coefficients[5];
void init_filter1(){
    int i;
    for(i=0;i<5;i++){
        filter1_coefficients[i]=(int)(f_filter_coefficients[i]*(2<<(N-1))-1);
        pc.printf("%d ",filter1_coefficients[i]);
    }
    pc.printf("\n\r");
}
void init_filter2(){
    int i;
    for(i=0;i<5;i++){
        filter2_coefficients[i]=(int)(f_filter_coefficients[i+5]*(2<<(N-1))-1);
        pc.printf("%d ",filter2_coefficients[i]);
    }
    pc.printf("\n\r");
}

//1rst filter
inline int filter1IIR( int16_t val ){
	// Read state variables
	static int w0,w1,w2;
    int32_t accumulator;
 	accumulator  = (w2 * filter1_coefficients[4])>>N;
	accumulator += (w1 * filter1_coefficients[3])>>N;
	accumulator += val ;   
	w0 = accumulator;	
		// Run feedforward part of filter
	accumulator  = (w0 * filter1_coefficients[0])>>N;
	accumulator += (w1 * filter1_coefficients[1])>>N;
	accumulator += (w2 * filter1_coefficients[2])>>N;
		w2 = w1;		// Shuffle history buffer
		w1 = w0;
        if(accumulator<0)
            accumulator=0;
//    pc.printf("w1=%d,w2=%d, acc=%d  ",w1,w2,accumulator);
    return accumulator;
}
//second filter
inline int filter2IIR( int16_t val ){
	// Read state variables
	static int w0,w1,w2;
    int32_t accumulator;
 	accumulator  = (w2 * filter2_coefficients[4])>>N;
	accumulator += (w1 * filter2_coefficients[3])>>N;
	accumulator += val ;   
	w0 = accumulator;	
		// Run feedforward part of filter
	accumulator  = (w0 * filter2_coefficients[0])>>N;
	accumulator += (w1 * filter2_coefficients[1])>>N;
	accumulator += (w2 * filter2_coefficients[2])>>N;
		w2 = w1;		// Shuffle history buffer
		w1 = w0;
        if(accumulator<0)
            accumulator=0;
//    pc.printf("w1=%d,w2=%d, acc=%d  ",w1,w2,accumulator);
    return accumulator;
}

main(){    
    int16_t val,val_tmp,val_filtered;
    ADCInitMaxSpeed();// 8bits max max speed...
    int channel=adcSelect(PortB,0);
    dacInit(DAC_LOW_POWER_OFF);
    init_filter1(); // float to int
    init_filter2(); // float to int
     while (1){
        val=adcRead(channel)<<4;// for speed up converting 8 bits only
        val_tmp=filter1IIR(val); //first filter
        val_filtered=filter2IIR(val_tmp);// seconde filter cascade
        dacWrite(val_filtered);// DAC on PTE30 / 12 bits
    //    pc.printf("%d\n\r",val);
        fgpioToggle(FPTD,4); // visu temps de conversion 5us
    }
} 


