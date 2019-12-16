### Features
- drivers for board mbed   [FRDM-KL25z](https://os.mbed.com/handbook/mbed-FRDM-KL25Z "mbed board Freescale FRDM kl25z") for processor **MKL25Z128VLK4**
- speed up latency from Interrupt GPIO (Port A or PortD) from **5.4us** from **InterruptIn** mbed Class to **1.2us** with** kl25_gpio.cpp** driver
- speed up DAC from **1.8us** with **AnalogOut** Class to **0.8us** with **kl25_dac.cpp** driver
- speed up ADC from 35us with **AnalogIn** Class to **2.9us** with **kl25_adc.cpp**
- add timer function with **kl25_Systick.cpp** (delay_ms and delay_us) with more accuracy when delay is in us than wait_ms and wait_us which use LPTMR 16 bits Timer.
- add kl25_cmp.cpp for use of comparator. More options than [Comparator Class](https://os.mbed.com/users/frankvnk/code/ComparatorIn/"library Comparator for mbed") with interruptions.
- add kl25_lptmr.cpp with interrupt or wait functions. Can't be use with wait mbed function.
- add kl25_sleep.cpp for use of VLPR mode and LLS sleep function. Wake up from pin or internal. Lots of example inside cpp function.
- add tpm0,1 and 2 Timer drivers for speed. for example it's possible to create a PWM signal 480kHz (from 0 to 100). Impossible with PwmOut Class.
- add tsi driver, more options than in [TSI Class](https://os.mbed.com/users/quevedo/code/TSIHW/"TSI Class")
- add debug driver in replace of Serial Class (minimal size and speed up), tests done until 1MHz (Class Serial is ok too)
-  main with a lot of tests inside (IIR FIR filter, test ADC DAC,...). A lots of programs are in cpp drivers and have just to be copied to main.cpp function.
### How to use
- Install [VS code](https://code.visualstudio.com/)
- add plugin **Platformio** in VS Code
- create a new project from Home with FRDM-KL25z board and mbed framework
- copy all files in src directory inside src directory in your folder inside platformio.
- enjoy
### Examples

Test IRQ better than InterruptIn test (5,8us)

    /*     Exemple test IRQ lattence = 1,2us*/
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

test of systick 1us toggle with delay_us = 1,3us better than wait_us (8us)

    #include "mbed.h"
    #include "kl25_systick.h"
    int main(void)
    {
        gpioDigitalOut(D3,0);// D3 = PTA12
        while (1)
        {
            delay_us(1);
            fgpioToggle(FPTA,12);
        }
    }
PWM 480kHz

    /* Exemple 3 : init generation PWM 480kHz 50% sur broche D4 et 10% sur D9
    * increment on channel1 with ISR
    * decrement on channel2 in main */
    #include "mbed.h"
    #include "kl25_tpm0.h"
    #include "kl25_systick.h"

    int channel1,channel2;
    void ISR(){
        static int  cpt1;
            tpm0SetPWM(channel1,cpt1+=5);
            if(cpt1>99)
                cpt1=0;
    }

    int main(void)
    {
        int cpt2=0;
        tpm0Init(TPM_DIV_1, 100);
        tpm0Start(TPM_CLK_PLL_48MHz); 
        channel1=tpm0EnablePWM(D4);//D4 pin
        printf("PD4=channel=%d\n\r",channel1);
        tpm0SetPWM(channel1,50);

        channel2=tpm0EnablePWM(D9);//D9 pin
        printf("PD5=channel=%d\n\r",channel2);
        tpm0SetPWM(channel2,10);
        tpm0_attachInterrupt(ISR);
        while (1)
        {
        if(cpt2<0)
                cpt2=99;
            tpm0SetPWM(channel2,cpt2-=5);
            delay_us(1); // this is not 1us cause of interrupt
        }
    }
Test comparator

    /* input on PTE29 (sinus) avec test with Vref/2 -> signal carre on D2*/

    #include "mbed.h"
    #include "kl25_cmp.h"
    #include "kl25_gpio.h"

    volatile bool gFlag=false;

    void ISR(){  
        gFlag=true;
        fgpioToggle(FPTD,4); // D2 out  
    }

    Serial pc(USBTX,USBRX);
    DigitalOut d2(PTD4);

    int main(void){
    cmpInit(CMP_hysteresis_5mV);
    cmpSetDacRef(32);
    cmpSetCompare(PTE29,NC);
    cmpEnableInterrupt(CMP_CHANGE);
    cmp_attachInterrupt(ISR);
        while(true){
            if(gFlag){
                printf("flag ");
                gFlag=false;
            }
        }
        return 0;
    }

Test ADC with AnalogIn time conversion = 35us

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
    
Test with ADC driver : better , time conversion = 2,9us
	
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
    }

Test timer Frequence meter max 70kHz on D3 InterruptIn

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

Test Timer Frequence meter max 270kHz signal on D3 : better than Interruptin

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
        if (gpioIsInterruptPending(PORTA,12)) {
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
Frequence meter on  External count pin on PE29 : up to 10MHz : The Best

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

Example read TSI in interrupt

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

ADC DAC Filter IIR ordre 4 freq sample = 200kHz, Freq stop = 20kHz

    // IIR Filter Design fs=200kHz
    //MicroModeler DSP http://www.micromodeler.com/launch.jsp

    #include "mbed.h"
    #include "kl25_adc.h"
    #include "kl25_dac.h"
    #include "kl25_gpio.h"

    Serial pc(USBTX,USBRX,115200);
    DigitalOut d2(D2);//PTD4

    float f_filter_coefficients[10] =     {
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
    
Example with sleep LLS ...

    /* exemple with 4 sources wakeup*/
    #include "mbed.h"
    #include "kl25_lptmr.h"
    #include "kl25_gpio.h"
    #include "kl25_sleep.h"
    #include "kl25_cmp.h"

    volatile bool gFlagLPTMR0,gFlagLLW,gFlagInterruptPin,gFlagCMP;
    Serial pc(USBTX, USBRX);
    DigitalIn d2(PTD4);
    DigitalOut d3(PTA12); 
    DigitalOut red(LED_RED);
    DigitalOut green(LED_GREEN);
    DigitalOut blue(LED_BLUE);
    //DigitalIn a0 (A0);

    void ISR_LPTMR0(){
        gFlagLPTMR0=true;
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

    volatile int a=0;//wait() can't be used cause lptmr used.

    int main(void){
        sleepInit();
        green=1;
        red=1;
        blue=1;
        gpioDigitalIn(PTD6);
    //  gpioEnableCLK(PortD); // D6
    //   gpioInitIn(FPTD,6); //D6 in
    //   gpioEnablePin(PORTD,6);//D6  
        gpioMode(PORTD,6,GPIO_MODE_PullUp); //D6 pullup
        gpioPortDEnableInterrupt(6,GPIO_MODE_INTERRUPT_CHANGE);

        d2.mode(PullUp);
        gpioPortDEnableInterrupt(4,GPIO_MODE_INTERRUPT_CHANGE);
        gpioPortD_attachInterrupt(ISR_PTD);

    cmpInit(CMP_hysteresis_5mV);
    cmpSetDacRef(32);
    cmpSetCompare(PTE29,NC);
    cmpEnableInterrupt(CMP_CHANGE);
    cmp_attachInterrupt(ISR_CMP);

    lptmrInit(LPTMR_DIV_8,500);
    lptmrStart(LPTMR_CLK_1kHz_LPO); //OK
    lptmr_attachInterrupt(ISR_LPTMR0);

        sleepPinEnable(PTD6,SLEEP_WAKUP_CHANGE);
        sleepPinEnable(PTD4,SLEEP_WAKUP_CHANGE);
    sleepSrcEnable(SLEEP_WKUP_FROM_LPTMR);
    sleepSrcEnable(SLEEP_WKUP_FROM_CMP);
        while (true)
        {
            pc.printf("SLEEP\n\r");
    //        wait_ms(10); /reconfigure Lptmr 
    //        pc.printf("end of wait\n\r");
            for(a=0;a<10000;a++);
            sleepLLS();
            pc.printf("RETURN FROM SLEEP\n\r");
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
        }
        return 0;
    }

