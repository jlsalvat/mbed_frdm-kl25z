#if !defined(TMP0_H_)
#define TMP0_H_   

#include "pinmap.h"
#include "kl25_util.h"
//hysteresis choice
//CR0[HYSTCTR] = 00  5mV
//CR0[HYSTCTR] = 01  10mV
//CR0[HYSTCTR] = 10  20mV
//CR0[HYSTCTR] = 11  30mV

typedef enum{
    CMP_hysteresis_5mV=0,
    CMP_hysteresis_10mV= 1,
    CMP_hysteresis_20mV= 2,
    CMP_hysteresis_30mV= 3
}CMPHystValue;

typedef enum {
    CMP0_IN0  =  0,
    CMP0_IN1  =  1,
    CMP0_IN2  =  2,
    CMP0_IN3  =  3,
    CMP0_IN4  =  4,
    CMP0_IN5  =  5,
    CMP0_IN6  =  6,
    CMP0_IN7  =  7
} CMPInputName;

const PinMap PinMap_CMP[] = {
    {PTE29, CMP0_IN5,0},
    {PTE30, CMP0_IN4,0},
    {PTC6,  CMP0_IN0,0},
    {PTC7, CMP0_IN1,0},
    {PTC8, CMP0_IN2,0},
    {PTC9, CMP0_IN3,0},
    {NC,    NC,0}
};

enum CmpModeInterrupt
{
    CMP_RISING = 1,
    CMP_FALLING = 2,
    CMP_CHANGE = 3
};

void cmpInit(CMPHystValue hysteresis );
void cmpSetDacRef(int value);
void cmpSetCompare(PinName pinP, PinName pinN);
//void cmpConfPin(PinName pin);

void cmpEnableInterrupt(CmpModeInterrupt mode);
void cmpDisableInterrupt();
void cmp_attachCMP0_IRQHandler();
void cmp_attachInterrupt(void (*userFunc)(void));
void cmp_dettachInterrupt();

inline void cmpEnable(){
    CMP0->CR1 = CMP_CR1_EN_MASK;// enable CMP
}

inline void cmpDisable(){
    CMP0->CR1 &= ~CMP_CR1_EN_MASK;// disable CMP
}

#endif