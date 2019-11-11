#include "kl25_gpio.h"
/***************************
 * Exemple D2 INPUT pull up D3 et LED RED output If D2=0 set D2 et led RED else RESET
 int main(){
    gpioEnableCLK(PortA); // D3
    gpioEnableCLK(PortB); // LED RED
    gpioEnableCLK(PortD); // D2
    gpioInitOut(FPTA,12,0); //D3 out=0
    gpioInitOut(FPTB,18,1); //LED Red out=0
    gpioInitIn(FPTD,4); //D2 in
    gpioEnablePin(PORTA,12); 
    gpioEnablePin(PORTB,18); //led RED
    gpioEnablePin(PORTD,4);   
    gpioMode(PORTD,4,GPIO_MODE_PullUp); //D2 pullup
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
    }
}
*************************************************************/

/************************************************************
 * use of interrupts on D2 and D3

bool gFlagD2,gFlagD3;
void ISR_D2(){
    gFlagD2=true;
}
void ISR_D3(){
    if(PORTA->ISFR & (1<<12)) // pin 12 ?
        gFlagD3=true;
    if(PORTA->ISFR & (1<<13)) // pin 13 ?
        //nothing to do
}
 int main(){
     // enable clock for ports
    gpioEnableCLK(PortA); // D3
    gpioEnableCLK(PortB); // LED RED
    gpioEnableCLK(PortD); // D2
    // init pin in or out
    gpioInitIn(FPTA,12); //D3 in
    gpioInitIn(FPTD,4); //D2 in
    gpioInitOut(FPTB,18,1); //LED Red out no light
    // enable pin (MUX)
    gpioEnablePin(PORTA,12); 
    gpioEnablePin(PORTB,18); //led RED
    gpioEnablePin(PORTD,4);  
    //input Pullup
    gpioMode(PORTD,4,GPIO_MODE_PullUp); //D2 pullup
    gpioMode(PORTA,12,GPIO_MODE_PullUp); //D3 pullup
    // external interrupt
    gpioPortAEnableInterrupt(12,GPIO_MODE_INTERRUPT_CHANGE);
    gpioPortAEnableInterrupt(13,GPIO_MODE_INTERRUPT_CHANGE);
    gpioPortDEnableInterrupt(4,GPIO_MODE_INTERRUPT_FALLING);
    gpioPortA_attachInterrupt(ISR_D3);
    gpioPortD_attachInterrupt(ISR_D2);
    while(1){
        if(gFlagD3){
            fgpioToggle(FPTB,18);
             printf("D3\n\r");
             gFlagD3=false;
        }
        if(gFlagD2){
            fgpioToggle(FPTB,18);
            printf("D2\n\r");
            gFlagD2=false;
        }
    }
}
*************************************************************/
/***********************************************************
 * Exemple test IRQ interruptIn IRQ lattence = 5,4us
 * time in IRQ is approximatively 8us
#include "mbed.h"
#include "kl25_gpio.h"
volatile bool gFlag=false;
void ISR(){  //delay 5.4us
    gFlag=true;
    fgpioToggle(FPTD,4); // D2 out  
}
Serial pc(USBTX,USBRX);
DigitalOut d2(PTD4);
InterruptIn d3(PTA12);
 int main(void){
     d3.mode(PullUp);
     d3.rise(ISR);
     while(true){
         if(gFlag){
             printf("flag ");
             gFlag=false;
         }
     }
     return 0;
}
************************************************************/
/**********************************************************
 * Exemple test IRQ lattence = 1,2us
#include "mbed.h"
#include "kl25_gpio.h"
volatile bool gFlag=false;
void ISR_PORTA(){ // delay 1.2us
    gFlag=true;
    // The "event" is connected to pin PTA12
    // so lets trap that and ignore all other GPIO interrupts.    
    // Test for IRQ on Port0.
    if (PORTA->ISFR & (1<<12)){
        PORTA->ISFR = (1<<12);
        fgpioToggle(FPTD,4); // D2 out
    } 

}
Serial pc(USBTX,USBRX);
DigitalOut d2(PTD4);
InterruptIn d3(PTA12);
 int main(void){
     d3.mode(PullUp);
     gpioPortAEnableInterrupt(12,GPIO_MODE_INTERRUPT_RISING);
     gpioPortA_attachInterrupt(ISR_PORTA);
     while(true){
         if(gFlag){
             printf("flag ");
             gFlag=false;
         }
     }
     return 0;
}
**********************************************************/
/*********************************************************
 * Exemple test IRQ lattence = 1us
#include "mbed.h"
#include "kl25_gpio.h"
volatile bool gFlag=false;
extern "C" void PORTA_IRQHandler  (void) {
    gFlag=true;
    // The "event" is connected to pin PTA12
    // so lets trap that and ignore all other GPIO interrupts.    
    // Test for IRQ on Port0.
    if (PORTA->ISFR & (1<<12)) {            
        PORTA->ISFR = (1<<12); // Clear pin interrupt
        fgpioToggle(FPTD,4); // D2 out
    }   
}
Serial pc(USBTX,USBRX);
DigitalIn d3(PTA12); //interrupt PTD4 in
DigitalOut d2(PTD4);
 int main(void){
     d3.mode(PullUp);
     gpioPortAEnableInterrupt(12,GPIO_MODE_INTERRUPT_RISING);
     gpioPort_attachPORTA_IRQHandler(); // delay 1us
     while(true){
         if(gFlag){
             printf("flag ");
             gFlag=false;
         }
     }
     return 0;
}
*************************************************************/
/************************************************************
 * test all interrupts
#include "mbed.h"
#include "kl25_gpio.h"

volatile bool gFlag=false;

extern "C" void PORTD_IRQHandler  (void) {
    gFlag=true;
    // The "event" is connected to pin PTA12
    // so lets trap that and ignore all other GPIO interrupts.    
    // Test for IRQ on Port0.
    if (PORTD->ISFR & (1<<5)) 
    {
        fgpioToggle(FPTD,4); // D2 out
    } 
    // Clear pin interrupt
    PORTD->ISFR = (1<<5);   

}
void ISR_PORTA(){ // delay 1.2us
    gFlag=true;
    // The "event" is connected to pin PTA12
    // so lets trap that and ignore all other GPIO interrupts.    
    // Test for IRQ on Port0.
    if (gpioIsInterruptPending(PORTA,4)) 
    {
        fgpioToggle(FPTD,4); // D2 out
        gpioClearInterrupt(PORTA,4); 
    } 
    if (gpioIsInterruptPending(PORTA,12)) 
    {
        fgpioToggle(FPTD,4); // D2 out
        gpioClearInterrupt(PORTA,12);  
    } 
}

Serial pc(USBTX,USBRX,115200);
DigitalIn d9(PTD5); //interrupt PTD4 in
DigitalOut d2(PTD4);
InterruptIn d3(PTA12);
InterruptIn d4(PTA4);
 int main(void){
     d3.mode(PullUp);
     d4.mode(PullUp);
     d9.mode(PullUp);
     gpioPortAEnableInterrupt(12,GPIO_MODE_INTERRUPT_RISING);
     gpioPortAEnableInterrupt(4,GPIO_MODE_INTERRUPT_RISING);
     gpioPortDEnableInterrupt(5,GPIO_MODE_INTERRUPT_RISING);
     gpioPort_attachPORTD_IRQHandler(); // delay 1us
     gpioPortA_attachInterrupt(ISR_PORTA);// delay 1,2us
     while(true){
         if(gFlag){
             printf("flag ");
             gFlag=false;
         }
     }
     return 0;
}
******************************************************************/

int gPinInterruptsPortA; //which pins of portA is enabled for interrupt
int gPinInterruptsPortD; //which pins of portD is enabled for interrupt
int gCptA,gCptD;// numbeurs of enabled pin interrupt

void (*fPORTA)(void);
void (*fPORTD)(void);

void PORTA_IRQ_Handler()
{
    fPORTA();
//reset flag have to be done in main fPORTA function...

// it's the solution to find location of pin to clear : too much time to compute... 
//    uint8_t location;
//    if(gCptA==1) // one only interrupt ?
//        PORTA->ISFR |= 1<<gPinInterruptsPortA; // clear interrupt
//    else{       //more than one interrupt ? it's longer to clear...
//        location = gpioSearchPinLocation(PORTA);
//        PORTA->ISFR = 1 << location; // clear interrupt of  pin detected
//    }
}

void PORTD_IRQ_Handler()
{
    fPORTD();
}

//same as DigitalIn class but faster
void gpioDigitalIn(PinName pin)
{
    unsigned int port = (unsigned int)pin >> PORT_SHIFT;
    unsigned int pin_n  = (pin&0xFF)>>2;
    printf("port=%d, pn_n=%d\n\r",port,pin_n);
    gpioEnableCLK((PortName)port);
    FGPIO_Type *fgpioReg = (FGPIO_Type *)(FPTA_BASE + port * 0x40);
    gpioInitIn(fgpioReg, pin_n);
    PORT_Type *port_reg = (PORT_Type *)(PORTA_BASE + 0x1000 *port);
    gpioEnablePin(port_reg, pin_n);
}
//same as DigitalOut class but faster
void gpioDigitalOut(PinName pin, int value)
{
    unsigned int port = (unsigned int)pin >> PORT_SHIFT;
    unsigned int pin_n  = (pin&0xFF)>>2;
    printf("port=%d, pn_n=%d\n\r",port,pin_n);
    gpioEnableCLK((PortName)port);
    FGPIO_Type *fgpioReg = (FGPIO_Type *)(FPTA_BASE + port * 0x40);
    gpioInitOut(fgpioReg, pin_n, value);
    PORT_Type *port_reg = (PORT_Type *)(PORTA_BASE + 0x1000 *port);
    gpioEnablePin(port_reg, pin_n);
}
// INTERRUPT PORTA
//Enable NVIC and attach userfunction in RAM to PORTA_IRQ_Handler
void gpioPortA_attachInterrupt(void (*userFunc)(void))
{
    fPORTA = userFunc;
    NVIC_SetVector(PORTA_IRQn, (uint32_t)PORTA_IRQ_Handler);
    NVIC_EnableIRQ(PORTA_IRQn);
    __enable_irq();
}
//Enable NVIC for PORTA_IRQ_Handler (FLASH)
void gpioPort_attachPORTA_IRQHandler()
{
    NVIC_EnableIRQ(PORTA_IRQn);
    __enable_irq();
}
//choose mode interrupt for pin of PORTA
void gpioPortAEnableInterrupt(int pin, ModeInterrupt mode)
{
    PORTA->PCR[pin] &= ~PORT_PCR_IRQC_MASK; //RAZ IRQC
    PORTA->PCR[pin] |= PORT_PCR_IRQC(mode); // choose type of Interrupt on Pin
}
//disable interrupt for pin of PORTA
void gpioPortADisableInterrupt(int pin)
{
    PORTA->PCR[pin] &= ~PORT_PCR_IRQC_MASK; //RAZ IRQC
// not tested but too long for interrupt
//    gCptA--;
//    if(gCptA==1){//need to know whish pin is activate
//        printf("disable");
//        gPinInterruptsPortA=gpioSearchPinEnabled(PORTA);// to be faster in IRQ
//   }
}
//disable all interrupts for pins of PORTA
void gpioPortA_dettachInterrupt()
{
    NVIC_DisableIRQ(PORTA_IRQn);
}
// INTERRUPT PORTD
//Enable NVIC and attach userfunction in RAM to PORTD_IRQ_Handler
void gpioPortD_attachInterrupt(void (*userFunc)(void))
{
    fPORTD = userFunc;
    NVIC_SetVector(PORTD_IRQn, (uint32_t)PORTD_IRQ_Handler);
    NVIC_EnableIRQ(PORTD_IRQn);
    __enable_irq();
}
//Enable NVIC for PORTD_IRQ_Handler (FLASH)
void gpioPort_attachPORTD_IRQHandler()
{
    NVIC_EnableIRQ(PORTD_IRQn);
    __enable_irq();
}
//choose mode interrupt for pin of PORTD
void gpioPortDEnableInterrupt(int pin, ModeInterrupt mode)
{
    PORTD->PCR[pin] &= ~PORT_PCR_IRQC_MASK; //RAZ IRQC
    PORTD->PCR[pin] |= PORT_PCR_IRQC(mode); // choose type of Interrupt on Pin
}
//disable interrupt for pin of PORTD
void gpioPortDDisableInterrupt(int pin)
{
    PORTD->PCR[pin] &= ~PORT_PCR_IRQC_MASK; //RAZ IRQC
}
//disable all interrupts for pins of PORTD
void gpioPortD_dettachInterrupt()
{
    NVIC_DisableIRQ(PORTD_IRQn);
}
//Enable clock SIM for Port
void gpioEnableCLK(PortName port)
{
    switch (port)
    {
    case PortA:
        SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
        break;
    case PortB:
        SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
        break;
    case PortC:
        SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
        break;
    case PortD:
        SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
        break;
    case PortE:
        SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
        break;
    }
}
// PDDR output and set value to PDOR register of FPTi register
void gpioInitOut(FGPIO_Type *fast_gpio_port, int pin, bool value)
{
    fast_gpio_port->PDDR |= (1 << pin); // output
    if (value == true)
        fast_gpio_port->PDOR |= (1 << pin);
    else
        fast_gpio_port->PDOR &= (1 << pin);
}
//PDDR input of FPTi register
void gpioInitIn(FGPIO_Type *fast_gpio_port, int pin)
{
    fast_gpio_port->PDDR &= ~(1 << pin); // input
}

//choosing Pullup, PullDown or PullNone for pin of PORTi
void gpioMode(PORT_Type *gpio_port, int pin, Mode mode)
{
    switch (mode)
    {
    case GPIO_MODE_PullUp:
        gpio_port->PCR[pin] |= 1 << PORT_PCR_PE_SHIFT; //PUll enable
        gpio_port->PCR[pin] |= 1 << PORT_PCR_PS_SHIFT; //PUll up
        break;
    case GPIO_MODE_PullDown:
        gpio_port->PCR[pin] |= 1 << PORT_PCR_PE_SHIFT;    //PUll enable
        gpio_port->PCR[pin] &= ~(1 << PORT_PCR_PS_SHIFT); //PUll Down
        break;
    case GPIO_MODE_PullNone:
        gpio_port->PCR[pin] &= ~(1 << PORT_PCR_PE_SHIFT); //PUll disable
    }
}
//Enable pin (ALT=1) for digital In or Out of PORTi
void gpioEnablePin(PORT_Type *gpio_port, int pin)
{
    gpio_port->PCR[pin] &= ~PORT_PCR_MUX_MASK;
    gpio_port->PCR[pin] |= PORT_PCR_MUX(1); // enable pin no alternate
}
