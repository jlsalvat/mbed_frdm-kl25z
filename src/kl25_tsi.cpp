#include "kl25_tsi.h"
#include "mbed.h"

/******************************************************************************
* Example 1 : put a cable on A0 pin and touch this pin for lighting red light
#include "mbed.h"
DigitalOut red(LED_RED);
int main(){
    uint16_t sense_A0;
    tsiInit(TSI_CURRENT_8uA, TSI_CURRENT_64uA, TSI_DV_1_03V, TSI_DIV_16, TSI_12_SCAN);
    TSIChannelName channel= tsiActivateChannel(A0);
    while(1) {
        sense_A0 = 0;
        tsiStart((TSIChannelName)channel);
        while(!tsiAvalaible()) {
            sense_A0= tsiRead();
            printf("TOUCH: %d\r\n", sense_A0);
        }
        if(sense_A0<380)
            red=1;
        else
            red=0;
    }
}
*********************************************************************/
/********************************************************************
 * Example read TSI in interrupt
#include "mbed.h"
DigitalOut red(LED_RED);
TSIChannelName gChannel;
void ISR_TSI0(){
    gFlagTSI=true;
    fgpioToggle(FPTA, 12); // D3 out   
    tsiStart(gChannel); 
}

int main(){
    uint16_t sense_A0;
    tsiInit(TSI_CURRENT_8uA, TSI_CURRENT_64uA, TSI_DV_1_03V, TSI_DIV_16, TSI_12_SCAN);
    gChannel= tsiActivateChannel(A0);
    tsiSetTreshold(375,330);
    tsiEnableInterrupt();
    tsiStart(gChannel);
    tsi_attachInterrupt(ISR_TSI0);
    while(1) {
        sense_A0 = 0;
        wait(0.1);
        if(gFlagTSI){
            sense_A0= tsiRead();
            printf("TOUCH: %d\r\n", sense_A0);
        if (red.read() == 0)
                red= 1;
            else
                red = 0;
        gFlagTSI=false;
        }
    }
}
**********************************************************/
void (*fTSI0)(void);

// RAM interrupt handler relocated
void TSI0_IRQ_Handler(){
    fTSI0();
   TSI0->GENCS |=TSI_GENCS_OUTRGF_MASK ;//clear TOF    
}


void tsiInit(TSI_Charge_Current refChg, TSI_Charge_Current extChg, TSI_DV dvolt, TSI_DIV_PRESCALER ps, TSI_NUMBER_OF_SCAN nscn) {
    // The first version is preconfigured for non-noise detection, no interrupts, not running on stop modes, software trigger
    SIM->SCGC5 |= SIM_SCGC5_TSI_MASK; // clock gate for TSI 
    TSI0->GENCS = 0xC0000080 | ((refChg & 0x07) << 21) | ((extChg & 0x07) << 16) | ((dvolt & 0x03) << 19) | ((ps & 0x07) << 13) | ((nscn & 0x1F) << 8);
}

/* Function to configure a pin to work with the corresponding channel (passed as the single parameter) */
TSIChannelName tsiActivateChannel(PinName pin) {
    // reads channel number and sets the MUX of the corresponding pin to ALT0 function
    unsigned int port = (unsigned int)pin >> PORT_SHIFT;
    unsigned int pin_n  = (pin&0xFF)>>2;
    switch(port){
        case 0: SIM->SCGC5|=SIM_SCGC5_PORTA_MASK; break;
        case 1: SIM->SCGC5|=SIM_SCGC5_PORTB_MASK; break;
        case 2: SIM->SCGC5|=SIM_SCGC5_PORTC_MASK; break;
    }
    TSIChannelName channel = (TSIChannelName)pinmap_peripheral(pin, PinMap_TSI);
    if(((int)channel)==NC)
        error("PinName provided to tsiActivateChannel() does not correspond to any known TSI channel.");
    printf("port=%d, pn_n=%d, channel=%d\n\r",port,pin_n,channel);
    PORT_Type *port_reg = (PORT_Type *)(PORTA_BASE + 0x1000 *port);  
    port_reg->PCR[pin_n] &= ~PORT_PCR_MUX_MASK;//choose ALT0 pin
    return channel;
}

// Function to trigger the reading of a given channel
void tsiStart(TSIChannelName channel) {
    //writes 1 to the software trigger bit, defining the channel number in the respective bits
    TSI0->GENCS |= TSI_GENCS_EOSF_MASK; // clears EOSF
    TSI0->DATA = TSI_DATA_SWTS_MASK  | TSI_DATA_TSICH(channel);
}

void tsiSetTreshold(uint16_t tresholdHigh,uint16_t tresholdLow){
    unsigned int tshd = (tresholdLow&0xFFFF) | (tresholdHigh<<16);
    TSI0->TSHD=tshd;
    printf("treshold=%x\n\r",tshd);
}

void tsiEnableInterruptOnTreshold(){
    TSI0->GENCS|=TSI_GENCS_TSIIEN_MASK;//The interrupt will wake MCU from low power mode if this interrupt is enabled.
    TSI0->GENCS|=TSI_GENCS_STPE_MASK;//This bit enables TSI module function in low power modes
    TSI0->GENCS&=~TSI_GENCS_ESOR_MASK;//0 Out-of-range interrupt is allowed.
}

void tsiEnableInterrupt(){
    TSI0->GENCS|=TSI_GENCS_TSIIEN_MASK; //This bit enables TSI module interrupt request to CPU when the scan completes
    TSI0->GENCS|=TSI_GENCS_ESOR_MASK;//1 End-of-scan interrupt is allowed.
}

void tsiDisableInterrupt(){
    TSI0->GENCS&=~TSI_GENCS_TSIIEN_MASK;//The interrupt will wake MCU from low power mode if this interrupt is enabled.
    TSI0->GENCS&=~TSI_GENCS_STPE_MASK;//This bit enables TSI module function in low power modes
}

void tsi_attachTSI0_IRQHandler(){
    NVIC_EnableIRQ(TSI0_IRQn);
    __enable_irq();
}

void tsi_attachInterrupt(void (*userFunc)(void)){
    fTSI0 = userFunc;
    NVIC_SetVector(TSI0_IRQn, (uint32_t)TSI0_IRQ_Handler);
    NVIC_EnableIRQ(TSI0_IRQn);
    __enable_irq();
}

void tsi_dettachInterrupt(){
    NVIC_DisableIRQ(TSI0_IRQn);
}
// Function to read scan result; returns zero if not finished
uint16_t tsiRead() {
    uint16_t aux;
    aux = TSI0->DATA & 0x0000FFFF;
    TSI0->GENCS |= TSI_GENCS_EOSF_MASK; // clears EOSF
    return aux;
}


