#include "kl25_sleep.h"
/* exemple with 4 sources wakeup
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
****************************************************************/
/*************************************************************
 * Test mode VLPR : problème printf bloquant...
 #include "mbed.h"
#include "kl25_sleep.h"
#include "kl25_systick.h"
DigitalOut d2(D2);
DigitalOut red(LED_RED);
Serial pc(USBTX,USBRX,9600);
int main(){
    sleepGoToVLPRunMode();// Go to 1MHz low power

    sleepChangeUart0_9600bps();
    while (true)
    {
       d2=1;
       red=1;
       delay_ms(1); //not 1ms but 1/48 ms
     //  pc.printf("1"); // bloquant!!!
       d2=0;
       red=0;
       delay_ms(1);
     //  pc.printf("0");
    }
    return 0;
}
*****************************************************************************/

/*
void (*fLLW)(void);

// RAM interrupt handler relocated
void LLW_IRQ_Handler(){
    if (PMC->REGSC & PMC_REGSC_ACKISO_MASK)
    {
        //If ACKISO flag is set, then write 1 to release IO pads and clocks 
        PMC->REGSC |= PMC_REGSC_ACKISO_MASK;
    }

    fLLW();
}

static void deepSleep(void)
{
    // Go to deep sleep mode when WFI instruction is called 
    SCB->SCR |=  SCB_SCR_SLEEPDEEP_Msk;
	// Dummy read 
	(void)SCB->SCR;
    // Go to sleep 
    __WFI();
}


void sleepAllowMode(SLEEP_EN_MODE en)
{
    // LLS VLLS or VLP mode enabled 
    SMC->PMPROT |= 1<<en;
}

void SMC_setVLLSxMode(void)
{
    // Clear VLLSx mode before writing the new value 
    SMC->STOPCTRL &= ~SMC_STOPCTRL_VLLSM_MASK;
    // Set VLLS mode 
    SMC->STOPCTRL |= SMC_STOPCTRL_VLLSM(SLEEP_VLLS_MODE);
    // Specify that VLLSx is selected 
    SMC->PMCTRL = SMC_PMCTRL_STOPM(4);

    deepSleep();

    // Check if previous STOP mode was entered correctly 
    if (SMC->PMCTRL & SMC_PMCTRL_STOPA_MASK)
    {
        // Error on previous STOP mode entry 
    }
}
*/
/*
uint16_t sleepGetSrcReset(){
    uint16_t u16_resetFlags = 0;
    // SRS0  Register 
    if (RCM->SRS0 & RCM_SRS0_WAKEUP_MASK)
    {
        u16_resetFlags |= SLEEP_RST_WakeUpSource;
    }
    if (RCM->SRS0 & RCM_SRS0_LVD_MASK)
    {
        u16_resetFlags |= SLEEP_RST_LVDSource;
    }
    if (RCM->SRS0 & RCM_SRS0_LOC_MASK)
    {
        u16_resetFlags |= SLEEP_RST_LossOfClockSource;
    }
    if (RCM->SRS0 & RCM_SRS0_LOL_MASK)
    {
        u16_resetFlags |= SLEEP_RST_LossOfLockSource;
    }
    if (RCM->SRS0 & RCM_SRS0_WDOG_MASK)
    {
        u16_resetFlags |= SLEEP_RST_WatchDogSource;
    }
    if (RCM->SRS0 & RCM_SRS0_PIN_MASK)
    {
        u16_resetFlags |= SLEEP_RST_ExtPinSource;
    }
    if (RCM->SRS0 & RCM_SRS0_POR_MASK)
    {
        u16_resetFlags |= SLEEP_RST_PowerOnResetSource;
    }
    if (RCM->SRS1 & RCM_SRS1_LOCKUP_MASK)
    {
        u16_resetFlags |= SLEEP_RST_LockUpSource;
    }
    if (RCM->SRS1 & RCM_SRS1_SW_MASK)
    {
        u16_resetFlags |= SLEEP_RST_SoftwareSource;
    }
    if (RCM->SRS1 & RCM_SRS1_MDM_AP_MASK)
    {
        u16_resetFlags |= SLEEP_RST_MDM_ApSource;
    }
    if (RCM->SRS1 & RCM_SRS1_SACKERR_MASK)
    {
        u16_resetFlags |= SLEEP_RST_StopModeAckErrorSource;
    }
    return u16_resetFlags;
}
*/
/**************************************************************
 * Example with tsi, cmp, lptimer, pins wake up
 * 
#include "mbed.h"
#include "kl25_lptmr.h"
#include "kl25_gpio.h"
#include "kl25_sleep.h"
#include "kl25_cmp.h"
#include "kl25_tsi.h"

volatile bool gFlagLPTMR0,gFlagLLW,gFlagInterruptPin,gFlagCMP,gFlagTSI;
Serial pc(USBTX, USBRX);
DigitalIn ptb0(PTB0);
DigitalIn d2(PTD4);
DigitalOut d3(PTA12); 
DigitalOut red(LED_RED);
DigitalOut green(LED_GREEN);
DigitalOut blue(LED_BLUE);

TSIChannelName gChannel;

void ISR_TSI0(){
    gFlagTSI=true;
    fgpioToggle(FPTA, 12); // D3 out   
}
void ISR_LPTMR0(){

    gFlagLPTMR0=true;
    tsiStart(gChannel);
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

volatile int a=0;


int main(void)
{
    sleepInit();
    green=1;
    red=1;
    blue=1;
    ptb0.mode(PullUp);
    //PTD6
    gpioDigitalIn(PTD6);
    gpioMode(PORTD,6,GPIO_MODE_PullUp); //D6 pullup
    gpioPortDEnableInterrupt(6,GPIO_MODE_INTERRUPT_CHANGE);

    d2.mode(PullUp);
    gpioPortDEnableInterrupt(4,GPIO_MODE_INTERRUPT_CHANGE);
    gpioPortD_attachInterrupt(ISR_PTD);

    tsiInit(TSI_CURRENT_8uA, TSI_CURRENT_64uA, TSI_DV_1_03V, TSI_DIV_16, TSI_12_SCAN);
    gChannel=tsiActivateChannel(A0);
    tsiSetTreshold(400,80);
    tsiEnableInterruptOnTreshold();
    tsi_attachInterrupt(ISR_TSI0);

  cmpInit(CMP_hysteresis_5mV);
  cmpSetDacRef(32);
  cmpSetCompare(PTE29,NC);
  cmpEnableInterrupt(CMP_CHANGE);
  cmp_attachInterrupt(ISR_CMP);

 lptmrInit(LPTMR_DIV_8,100);
 lptmrStart(LPTMR_CLK_1kHz_LPO); //OK
 lptmr_attachInterrupt(ISR_LPTMR0);


//  sleepPinEnable(PTB0,SLEEP_WAKUP_CHANGE); Not working on PORTB and PORTC !!!!
     sleepPinEnable(PTD6,SLEEP_WAKUP_CHANGE);
    sleepPinEnable(PTD4,SLEEP_WAKUP_CHANGE);
 sleepSrcEnable(SLEEP_WKUP_FROM_LPTMR);
 sleepSrcEnable(SLEEP_WKUP_FROM_CMP);
 sleepSrcEnable(SLEEP_WKUP_FROM_TSI);
    while (true)
    {
       pc.printf("SLEEP\n\r");
        for(a=0;a<10000;a++);
        sleepLLS();
        pc.printf("RETURN FROM SLEEP\n\r");
        printf("tsiRead()=%d\n\r",tsiRead());
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
         if(gFlagTSI){
        pc.printf("INTERRUPT from TSI\n\r ");
        red=1;
        green=0;
        if (blue.read() == 0)
                blue = 1;
            else
                blue = 0;
        gFlagTSI=false;
        }
     }
    return 0;
}
**********************************************************************/
/**************************************************************
 * Exemple go to 1MHz VLPR for low power
 * 
 #include "mbed.h"
#include "kl25_sleep.h"
#include "kl25_systick.h"
DigitalOut d2(D2);
DigitalOut red(LED_RED);
int main(){
    sleepGoToVLPRunMode();// Go to 1MHz low power
    while (true)
    {
       d2=1;
       red=1;
       delay_ms(1); //not 1ms but 1/48 ms
       d2=0;
       red=0;
       delay_ms(1);
    }
    return 0;
}
*****************************************************************************************/
//enter LLS mode 2uA
void SMCSleepDeep(SLEEP_MODE mode)
{
  volatile unsigned int dummyread;

  // The PMPROT register allows the MCU to enter the LLS modes.
  SMC->PMPROT = SMC_PMPROT_ALLS_MASK;   

  // Set the STOPM field to 0b011 for LLS mode 
  SMC->PMCTRL &= ~SMC_PMCTRL_STOPM_MASK; 
  SMC->PMCTRL |=  SMC_PMCTRL_STOPM(mode); 

  //wait for write to complete to SMC before stopping core   
  dummyread = SMC->PMCTRL;
  dummyread++;

  // Set the SLEEPDEEP bit to enable deep sleep mode (STOP) 
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

  __WFI();
}

//Flag clear for Pin (Not used)
void sleepClearFlag(PinName pin){
      LLUPinName pe_n = (LLUPinName)pinmap_peripheral(pin, PinMap_LLWU); 
      if(pe_n<8)
            LLWU->F1|=1<<pe_n;
      else
      {
            LLWU->F2|=1<<(pe_n>>8);
      }
}
//get Flag  for Pin (Not used)
int8_t sleepGetFlag(PinName pin)
{
    LLUPinName pe_n = (LLUPinName)pinmap_peripheral(pin, PinMap_LLWU);
    int8_t u8_ret = -1;
    if (pe_n<8)
       return((LLWU->F1 & (1 << pe_n)) != 0);
    else
        return((LLWU->F2 & (1 << (pe_n>>8))) != 0);
}
// Function to LLS Sleep
void sleepLLS(void)
{
//LLW_IRQHandler 
  SMCSleepDeep(SLEEP_LLS_MODE);
}
//Enable internal src wakeup
void sleepSrcEnable(SLEEP_WKUP_SRC src){
    LLWU->ME|=1<<(src);
}
//Disable internal src wakeup
void sleepSrcDisable(SLEEP_WKUP_SRC src){
    LLWU->ME&=~(1<<(src));
}
// Enable a pin for use with the low leakage wake up unit
void sleepPinEnable(PinName pin, SLEEP_WKUP_PIN_MODE mode)
{    
    if(!(pin==PTD4 || pin==PTD2))
        printf("!!!! ONLY PTD4 or PTD6 can Wake up from SLEEP! ");
    LLUPinName pe_n = (LLUPinName)pinmap_peripheral(pin, PinMap_LLWU);
    if(pe_n < 4){
        LLWU->PE1 &= ~(0x3 << (pe_n*2));//RAZ pin before using
        switch(mode){
                case 0:                
                LLWU->PE1 |=     (0x0 << (pe_n*2)) ;
                break;
                case 1:
                LLWU->PE1 |=     (0x1 << (pe_n*2));
                break;
                case 2:
                LLWU->PE1 |=     (0x2 << (pe_n*2));
                break;
                case 3:
                LLWU->PE1 |=     (0x3 << (pe_n*2));
                break;    
            }    
    }
    else if((pe_n >= 4) && (pe_n < 8)){
        LLWU->PE2 &= ~(0x3 << ((pe_n-4)*2));//RAZ pin before using
        switch(mode){
                case 0:
                LLWU->PE2 |=     (0x0 << ((pe_n-4)*2));
                break;
                case 1:
                LLWU->PE2 |=     (0x1 << ((pe_n-4)*2));
                break;
                case 2:
                LLWU->PE2 |=     (0x2 << ((pe_n-4)*2));
                break;
                case 3:
                LLWU->PE2 |=     (0x3 << ((pe_n-4)*2));
                break;    
            }        
    }    
    else if((pe_n >= 8) && (pe_n < 12)){
        LLWU->PE3 &= ~(0x3 << ((pe_n-8)*2));//RAZ pin before using        
        switch(mode){
                case 0:
                LLWU->PE3 |=     (0x0 << ((pe_n-8)*2));
                break;
                case 1:
                LLWU->PE3 |=     (0x1 << ((pe_n-8)*2));
                break;
                case 2:
                LLWU->PE3 |=     (0x2 << ((pe_n-8)*2));
                break;
                case 3:
                LLWU->PE3 |=     (0x3 << ((pe_n-8)*2));
                break;        
            }    
    }
    else if((pe_n >= 12) && (pe_n < 16)){
        LLWU->PE3 &= ~(0x3 << ((pe_n-12)*2));//RAZ pin before using        
        switch(mode){
                case 0:
                LLWU->PE4 |=     (0x0 << ((pe_n-12)*2));
                break;
                case 1:
                LLWU->PE4 |=     (0x1 << ((pe_n-12)*2));
                break;
                case 2:
                LLWU->PE4 |=     (0x2 << ((pe_n-12)*2));
                break;
                case 3:
                LLWU->PE4 |=     (0x3 << ((pe_n-12)*2));
                break;    
            }    
    }
}
//init LLWU registers
void sleepInit(){
    //clear all interrupt flags 
    LLWU->F1 |= 0xFF;
    LLWU->F2 |= 0xFF;
    LLWU->F3 |= 0xFF;
    // Disable all external sources
    LLWU->PE1 &= 0;
    LLWU->PE2 &= 0;
    LLWU->PE3 &= 0;
    LLWU->PE4 &= 0;
    // Disable all internal sources 
    LLWU->ME &= 0;
}



void switchFEItoBLPI( void )
{
	/* Initialization */

    /* SIM->SCGC5: PORTA=1 */
    SIM->SCGC5 |= (uint32_t)0x0200UL;     /* Enable clock gate for ports to enable pin routing */

    /* SIM->CLKDIV1: OUTDIV1=1,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,OUTDIV4=1,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0 */
    SIM->CLKDIV1 = (uint32_t)0x10010000UL; /* Update system prescalers - core clock div by 2, bus clock div by 2 */

    SystemCoreClockUpdate();
    /* PORTA->PCR18: ISF=0,MUX=0 */
    PORTA->PCR[18] &= (uint32_t)~0x01000700UL;
    /* PORTA->PCR19: ISF=0,MUX=0 */
    PORTA->PCR[19] &= (uint32_t)~0x01000700UL;

    /* Switch to FBE Mode */

    /* OSC0->CR: ERCLKEN=1,??=0,EREFSTEN=0,??=0,SC2P=1,SC4P=0,SC8P=0,SC16P=1 */ //switch the external crystal on
    OSC0->CR = (uint8_t)0x89U;
    SystemCoreClockUpdate();

    /* MCG->C2: LOCRE0=0,??=0,RANGE0=1,HGO0=1,EREFS0=1,LP=0,IRCS=0 */
    MCG->C2 = (uint8_t)0x1CU;
    SystemCoreClockUpdate();

    /* MCG->C1: CLKS=2,FRDIV=2,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
    MCG->C1 = (uint8_t)0x82U;
    SystemCoreClockUpdate();

    /* MCG->C4: DMX32=0,DRST_DRS=0 */
    MCG->C4 &= (uint8_t)~(uint8_t)0xE0U;

    /* MCG->C5: ??=0,PLLCLKEN0=0,PLLSTEN0=0,PRDIV0=1 */
    MCG->C5 = (uint8_t)0x01U;

    /* MCG->C6: LOLIE0=0,PLLS=0,CME0=0,VDIV0=0 */
    MCG->C6 = (uint8_t)0x00U;

    while((MCG->S & MCG_S_OSCINIT0_MASK) == 0x00U){//wait till crystal selected by C2 eREFS0 has been initialized (1)
    }

    while((MCG->S & MCG_S_IREFST_MASK) != 0x00U) { /* Check that the source of the FLL reference clock is the external reference clock. */
    }
    while((MCG->S & 0x0CU) != 0x08U) {    /* Wait until external reference clock is selected as MCG output */
    }


    	/* Move to FBI Mode */


    	/* MCG->C1: CLKS=1,FRDIV=2,IREFS=1,IRCLKEN=O,IREFSTEN=0 */
    	MCG->C1 = (uint8_t)0x54U; /* Switch the system clock to the internal reference clock */

    	MCG->C2 |=(1<<0);//select fast internal reference clock


        while((MCG->S & MCG_S_IREFST_MASK) == 0x00U) {   /* Wait until IREFST is 1, indicating that internal reference clock has been selected as the reference clock source */
        }

        while((MCG->S & MCG_S_CLKST_MASK) != 0x1U << MCG_S_CLKST_SHIFT) {   /* Wait until the the internal reference clock is selected to feed MCGOUTCLK */
        }

        	/* Move to BLPI Mode */
        	/* MCG->C2: LP=1,IRCS=1 */
        	MCG->C2 = (uint8_t)0x03U;

        	SystemCoreClockUpdate();
}


void sleepGoToVLPRunMode(void)
{
	//Switch off the clock into SCGC6 register - FTF Flash memory, which prevents entry into low power mode
	SIM->SCGC6 &=~ (1 <<0);

	// Allow very low power modes, can be written into only ONCE!
    SMC->PMPROT |=SMC_PMPROT_AVLP_MASK;

    //Very-Low-Power Run mode (VLPR) entry (enable)
    SMC->PMCTRL |=  SMC_PMCTRL_RUNM(2);

	switchFEItoBLPI(); // Switch from FEI MCG mode to BLPI MCG mode, internal fast 4MHz clock -> core clock divided to 1MHz, bus clock to core clock/2
}

// bloquant en mode VLPR ???
void sleepChangeUart0_9600bps(){
SIM->SOPT2 |= SIM_SOPT2_UART0SRC(3);// use IRC clock output for UART Baud rate generator 
UART0->C2 = 0;// turn off UART0 while changing configurations 
UART0->BDH = 0x00;
UART0->BDL = 13; // 9600 Baud = 1000000/13/8
UART0->C4 = 0x07;
UART0->C2 = UART_C2_TE_MASK;// turn on TX
}




