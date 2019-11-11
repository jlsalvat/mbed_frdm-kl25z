#if !defined(DAC_H_)
#define DAC_H_  
#include "pinmap.h"
enum TSI_Charge_Current{
    TSI_CURRENT_500nA=0b000,
    TSI_CURRENT_1uA=0b001,
    TSI_CURRENT_2uA=0b010,
    TSI_CURRENT_4uA=0b011,
    TSI_CURRENT_8uA=0b100,
    TSI_CURRENT_16uA=0b101,
    TSI_CURRENT_32uA=0b110,
    TSI_CURRENT_64uA=0b111
};

enum TSI_DV{
    TSI_DV_1_03V=0b00, //VP = 1.33 V; Vm = 0.30 V.
    TSI_DV_0_73V=0b01, //VP = 1.18 V; Vm = 0.45 V.
    TSI_DV_0_43V=0b10, //VP = 1.03 V; Vm = 0.60 V.
    TSI_DV_0_29V=0b11  //VP = 0.95 V; Vm = 0.67 V
};

enum TSI_DIV_PRESCALER{
    TSI_DIV_1 = 0,
    TSI_DIV_2 = 1,
    TSI_DIV_4 = 2,
    TSI_DIV_8 = 3,
    TSI_DIV_16 = 4,
    TSI_DIV_32 = 5,
    TSI_DIV_64 = 6,
    TSI_DIV_128 = 7
};

enum TSI_NUMBER_OF_SCAN{
    TSI_1_SCAN = 0,
    TSI_2_SCAN = 1,
    TSI_3_SCAN = 2,
    TSI_4_SCAN = 3,
    TSI_5_SCAN = 4,
    TSI_6_SCAN = 5,
    TSI_7_SCAN = 6,
    TSI_8_SCAN = 7,
    TSI_9_SCAN = 8,
    TSI_10_SCAN = 9,
    TSI_11_SCAN = 10,
    TSI_12_SCAN = 11,
    TSI_13_SCAN = 12,
    TSI_14_SCAN = 13,
    TSI_15_SCAN = 14,
    TSI_16_SCAN = 15
};

typedef enum {
    TSI0_CH0  =  0,
    TSI0_CH1  =  1,
    TSI0_CH2  =  2,
    TSI0_CH3  =  3,
    TSI0_CH4  =  4,
    TSI0_CH5  =  5,
    TSI0_CH6  =  6,
    TSI0_CH7  =  7,
    TSI0_CH8  =  8,
    TSI0_CH9  =  9,
    TSI0_CH10  =  10,
    TSI0_CH11  =  11,
    TSI0_CH12  =  12,
    TSI0_CH13  =  13,
    TSI0_CH14  =  14,
    TSI0_CH15  =  15,

} TSIChannelName;

const PinMap PinMap_TSI[] = {
    {PTA0, TSI0_CH1,0},
    {PTA1, TSI0_CH2,0},
    {PTA2,  TSI0_CH3,0},
    {PTA3, TSI0_CH4,0},
    {PTA4, TSI0_CH5,0},
    {PTB0, TSI0_CH0,0},
    {PTB1, TSI0_CH6,0},
    {PTB2,  TSI0_CH7,0},
    {PTB3, TSI0_CH8,0},
    {PTB16, TSI0_CH9,0},
    {PTB17, TSI0_CH10,0},
    {PTB18, TSI0_CH11,0},
    {PTB19, TSI0_CH12,0},
    {PTC0, TSI0_CH13,0},
    {PTC1, TSI0_CH14,0},
    {PTC2, TSI0_CH15,0},
    {NC,    NC,0}
};

void tsiInit(TSI_Charge_Current refChg, TSI_Charge_Current extChg, TSI_DV dvolt, TSI_DIV_PRESCALER ps, TSI_NUMBER_OF_SCAN nscn);

/* Function to configure a pin to work with the corresponding channel (passed as the single parameter) */
TSIChannelName tsiActivateChannel(PinName pin);
void ActivateChannel(char ch) ;

// Function to trigger the reading of a given channel
void tsiStart(TSIChannelName channel);

void tsiSetTreshold(uint16_t tresholdHigh,uint16_t tresholdLow);

void tsiEnableInterruptOnTreshold();

void tsiEnableInterrupt();

void tsiDisableInterrupt();

void tsi_attachTSI0_IRQHandler();

void tsi_attachInterrupt(void (*userFunc)(void));

void tsi_dettachInterrupt();
// Function to read scan result; returns zero if not finished
uint16_t tsiRead();

bool inline  tsiAvalaible(){
   return ((TSI0->GENCS & TSI_GENCS_EOSF_MASK)==0);
}


#endif