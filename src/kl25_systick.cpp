#include "kl25_systick.h"
/***************************************
 * Exemple ISR every second change on GREEN LED 
 void ISR(){
    static int i;
    i++;
    if(i>1000){
        i=0;
        PTB->PTOR = 0x080000; 
    }

}
int main (void){
SIM->SCGC5 |= 0x400;
PORTB->PCR[19] = 0x100;
PTB->PDDR |= 0x080000;
systick_attachInterrupt((41940 - 1),ISR);
while (1) {
//delay_ms(1000);
//PTB->PTOR = 0x080000;
}
}
****************************************/

/*************************************
 * Exemple 2 : true 1us toggle LED better than delay_us
void ISR(){
    fgpioToggle(FPTA,12);
}
 int main(void)
{
    gpioDigitalOut(D3,0);// D3 = PTA12
    systick_attachInterrupt(48,ISR1);
    while (1)
    {
    }
}
***************************************/

/*************************************
 * Exemple 3 : 1us toggle with delay_us = 1,3us
 int main(void)
{
    gpioDigitalOut(D3,0);// D3 = PTA12
    while (1)
    {
        delay_us(1);
        fgpioToggle(FPTA,12);
    }
}
***************************************/
void (*fSysTick)(void);

void SysTick_IRQ_Handler(){
    fSysTick();
}

void delay_ms(int n){
int i;
SysTick->LOAD = 48000 - 1; // frequency of FRDMKL25z = 48MHz
SysTick->CTRL = SysTick_CTRL_ENABLE_Msk|SysTick_CTRL_CLKSOURCE_Msk  ; // enable Systick timer
for(i = 0; i < n; i++) {
while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0) // wait end flag
{ }
}
SysTick->CTRL = 0; // stop systick timer
}



void systick_attachInterrupt(int value, void (*userFunc)(void)){
fSysTick=userFunc;
SysTick->LOAD = value - 1; // frequency of FRDMKL25z = 41,94MHz
SysTick->CTRL = SysTick_CTRL_ENABLE_Msk|SysTick_CTRL_CLKSOURCE_Msk|SysTick_CTRL_TICKINT_Msk  ; // enable Systick timer and interrupt
NVIC_SetVector(SysTick_IRQn, (uint32_t)SysTick_IRQ_Handler);
NVIC_EnableIRQ(SysTick_IRQn);
}

void systick_dettachInterrupt(){
NVIC_DisableIRQ(SysTick_IRQn);
}