#if !defined(SYSTICK_H_)
#define SYSTICK_H_  
#include "mbed.h"
void delay_ms(int n);
void inline delay_us(int n){
int i;
SysTick->LOAD = 48 - 1; // frequency of FRDMKL25z = 48MHz
SysTick->CTRL = SysTick_CTRL_ENABLE_Msk|SysTick_CTRL_CLKSOURCE_Msk  ; // enable Systick timer
for(i = 0; i < n; i++) {
while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0) // wait end flag
{ }
}
SysTick->CTRL = 0; // stop systick timer
}
void systick_attachInterrupt(int value, void (*userFunc)(void));
void systick_dettachInterrupt();



#endif