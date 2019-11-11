#if !defined(LPTMR_H_)
#define LPTMR_H_   

#include "PeripheralPins.h"
#include "kl25_util.h"

enum LPTMR_Prescaler{
    LPTMR_DIV_2 = 0,
    LPTMR_DIV_4 = 1,
    LPTMR_DIV_8 = 2,
    LPTMR_DIV_16 = 3,
    LPTMR_DIV_32 = 4,
    LPTMR_DIV_64 = 5,
    LPTMR_DIV_128 = 6,
    LPTMR_DIV_256 = 7,
    LPTMR_DIV_512 = 8,
    LPTMR_DIV_1024 = 9,
    LPTMR_DIV_2048 = 10,
    LPTMR_DIV_4096 = 11,
    LPTMR_DIV_8192 = 12,
    LPTMR_DIV_16384 = 13,
    LPTMR_DIV_32768 = 14,
    LPTMR_DIV_65536 = 15
};
enum LPTMR_Clk_Sel{
    LPTMR_CLK_32kHz_INTERNAL = 0b00,//MCGIRCLK — internal reference clock (not available in LLS and VLLS modes)
    LPTMR_CLK_4MHz_INTERNAL =0b100,       
    LPTMR_CLK_1kHz_LPO=0b10, //ERCLK32K (not available in VLLS0) with LPO in ERCLK32K
    LPTMR_CLK_8MHz_EXT=0b11 //  OSCERCLK — external reference clock(not available in VLLS0 mode)
};

// functions tpm0
void lptmrInit(LPTMR_Prescaler div, int value);
//becarefull clock selection from SIM is for all Timers LPTMR0,1 and 2...
void lptmrStart(LPTMR_Clk_Sel clk);

//relocate LPTMR0_IRQHandler with NVIC CMIS function
void lptmr_attachInterrupt(void (*userFunc)(void));

// use LPTMR0_IRQHandler in main
void lptmr_attachLPTimer_IRQHandler ();

void lptmr_detachInterrupt();
// fast function lptmr0

void inline lptmrStop(){
    LPTMR0->CSR &= ~LPTMR_CSR_TEN_MASK; // disable LPTIMER
}

//not working...
/*int inline lptmrRead(){
    return LPTMR0->CNR;
}*/

int inline lptmrOverflow(){
    if((LPTMR0->CSR & LPTMR_CSR_TCF_MASK) == 0)
        return 0;
    else{
        LPTMR0->CSR |= LPTMR_CSR_TCF_MASK;/* clear TOF */
        return 1;
    }
}
#endif