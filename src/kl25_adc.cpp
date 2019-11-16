#include "kl25_adc.h"
#include "mbed.h"
/***************************************
 * Exemple adc conversion 
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
****************************************/

void adcInit(ADC_Low_Power lp,ADC_Size size,ADC_Sample_Time sampleTime,ADC_Avg avg){
    
    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK; // enable CLK ADC0

    if(lp==ADC_Low_Power_ON)
        ADC0->CFG1 = ADC_CFG1_ADLPC_MASK;  // Low-Power Configuration
    if(sampleTime>0){
        ADC0->CFG1 |= ADC_CFG1_ADLSMP_MASK;        
        ADC0->CFG2 = ADC_CFG2_ADLSTS(sampleTime&3);
    }

    ADC0->CFG1 |=ADC_CFG1_ADIV(1)    // Clock Divide Select: (Input Clock)/2 : 24MHz/2=12MHz
               | ADC_CFG1_MODE(size)               // (n)bits Resolution
               | ADC_CFG1_ADICLK(0);    // Input Clock: (Bus Clock=24MHz)

  /* ADC0_CFG2: ADACKEN=1,ADHSC=0,ADLSTS=0 for speed up */
  ADC0->CFG2 = (uint32_t)((ADC0->CFG2 & (uint32_t)~(uint32_t)(
               ADC_CFG2_ADHSC_MASK |
               ADC_CFG2_ADLSTS(0x03)
              )) | (uint32_t)(
               ADC_CFG2_ADACKEN_MASK
              ));

    if(avg>0)
    ADC0->SC3 = ADC_SC3_AVGE_MASK       // Hardware Average Enable
              | ADC_SC3_AVGS(avg&3);   // which number of avg samples
//   printf("CFG1=%x CFG2=%x CFG3=%x\n\r",ADC0->CFG1,ADC0->CFG2,ADC0->SC3);
}

void ADCInitMaxSpeed(){
    adcInit(ADC_Low_Power_ON,ADC_Size_8bits,ADC_Sample_Time_Minimal,ADC_No_Avg);
    ADC0->CFG1 &=~ADC_CFG1_ADIV_MASK;  // no div input =24MHz ??? ca fonctionne
}



int adcSelect(PortName port, int pin){
    int channel=-1;
    if(port==PortE && pin==20){
        SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;// enable clock
        PORTE->PCR[20]&= ~PORT_PCR_MUX_MASK;// Analog Input
        channel=0;        
    }
    if(port==PortE && pin==22){
        SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;// enable clock
        PORTE->PCR[22]&= ~PORT_PCR_MUX_MASK;// analog Input
        channel=3;   
    }
    if(port==PortE && pin==21){
        SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;// enable clock
        PORTE->PCR[21] &= ~PORT_PCR_MUX_MASK;// analog Input
        channel=4;   
    }
    if(port==PortD && pin==1){
        SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;// enable clock
        PORTD->PCR[1]&= ~PORT_PCR_MUX_MASK;// analog Input
        channel=5;   
    }
    if(port==PortD && pin==5){
        SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;// enable clock
        PORTD->PCR[5]&= ~PORT_PCR_MUX_MASK;// analog Input
        channel=6;   
    }
    if(port==PortD && pin==6){
        SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;// enable clock
        PORTD->PCR[6]&= ~PORT_PCR_MUX_MASK;// analog Input
        channel=7;   
    }
    if(port==PortB && pin==0){
        SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;// enable clock
        PORTB->PCR[0] &= ~PORT_PCR_MUX_MASK;// analog Input
         channel=8;   
    }
    if(port==PortB && pin==1){
        SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;// enable clock
        PORTB->PCR[1]&= ~PORT_PCR_MUX_MASK;// analog Input
        channel=9;   
    }
    if(port==PortC && pin==2){
        SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;// enable clock
        PORTC->PCR[2]&= ~PORT_PCR_MUX_MASK;// analog Input
        channel=11;   
    }
    if(port==PortB && pin==2){
        SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;// enable clock
        PORTB->PCR[2]&= ~PORT_PCR_MUX_MASK;// analog Input
        channel=12;   
    }
    if(port==PortB && pin==3){
        SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;// enable clock
        PORTB->PCR[3]&= ~PORT_PCR_MUX_MASK;// analog Input
        channel=13;   
    }
    if(port==PortC && pin==0){
        SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;// enable clock
        PORTC->PCR[0]&= ~PORT_PCR_MUX_MASK;// analog Input
        channel=14;   
    }
    if(port==PortC && pin==1){
        SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;// enable clock
        PORTC->PCR[1]&= ~PORT_PCR_MUX_MASK;// analog Input
        channel=15;   
    }
    if(port==PortE && pin==30){
        SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;// enable clock
        PORTE->PCR[30]&= ~PORT_PCR_MUX_MASK;// analog Input
        channel=23;   
    }
    return channel;
}
int adcSelectTemp(){
    return 26;
}

int adcSelectVref(){
    return 29;
}
uint16_t adcRead(int channel) {
    // start conversion
    if(channel>=0){
    ADC0->SC1[0] = ADC_SC1_ADCH(channel & ~(1 << CHANNELS_A_SHIFT));

    // Wait Conversion Complete
    while ((ADC0->SC1[0] & ADC_SC1_COCO_MASK) != ADC_SC1_COCO_MASK);

    // Return value
    return (uint16_t)ADC0->R[0];
    }
    return -1;
}

