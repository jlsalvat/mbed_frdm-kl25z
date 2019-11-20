/****************************************************
 * example speed tests 460800 bps
 * send cpt in hexadecimal and integer view
 * minimal size and max speed
#include "mbed.h"
#include "kl25_debug.h"
//Serial pc(USBTX,USBRX,115200);

main(){
    int cpt;
    char sz[16];
    debugInit(460800);
    while (true)
    {
        cpt++;
        debugString("cpt=0x");
       debugIntHex(cpt);
        debugString(" ");
        __itoa(cpt,sz,10);
        debugString(sz);
        debugString("\n\r");    
        wait_ms(10);    
    }    
}


***************************************************************/
/**************************************************************
 * with Serial : max speed : 230400bps (cause div/2)
#include "mbed.h"
#include "kl25_debug.h"
Serial pc(USBTX,USBRX,230400);

main(){
    int cpt;
    while (true)
    {
        cpt++;
        pc.printf("cpt=%x %d\n\r",cpt,cpt);    
        wait_ms(10);    
    }    
}
// test done at 1MHz with terminal in platformio at 1000000Hz
// pio device monitor -b1000000
***************************************************************/

#include "kl25_debug.h"
#include "pinmap.h"
#include "clk_freqs.h"

// init UART0
void debugInit(int speed){
    int pClk=48000000; // init don't work ???
    
//    if (mcgpllfll_frequency() != 0)   {                 //PLL/FLL is selected ? yes
        SIM->SOPT2 |= (1<<SIM_SOPT2_UART0SRC_SHIFT); //activate frequency from PLL = 48MHz
        pClk = mcgpllfll_frequency();   // get 48 MHz
//    }
//    else{ 
//            pClk = extosc_frequency(); 
//            SIM->SOPT2 |= (2<<SIM_SOPT2_UART0SRC_SHIFT);
//    }
    SIM->SCGC4 |= SIM_SCGC4_UART0_MASK; // enable UART0 clk
    // Disable UART before changing registers
    UART0->C2 &= ~(UARTLP_C2_RE_MASK | UARTLP_C2_TE_MASK);
    // Enable UART transmitter to ensure TX activity is finished
    UART0->C2 |= UARTLP_C2_TE_MASK;
    // Wait for TX activity to finish
    while(!(UART0->S1 & UARTLP_S1_TC_MASK));
    // Disbale UARTs again
    UART0->C2 &= ~(UARTLP_C2_RE_MASK | UARTLP_C2_TE_MASK);

    int baud=pClk/8/speed;
    UART0->BDH = (baud>>8)& 0x1f;
    UART0->BDL = baud&0xFF; 
    UART0->C4 = 0x07; // synchronizer on 8 clk
    UART0->C1 = 0x00;//8-bit data 
    UART0->C2 |= UARTLP_C2_TE_MASK;// turn on TX
 //  UART0->C2 |= UARTLP_C2_RE_MASK;// turn on RX

    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;   //enable clock for PORTA 
    PORTA->PCR[2] = PORT_PCR_MUX(2); // make PTA2 UART0_Tx pin 
}
