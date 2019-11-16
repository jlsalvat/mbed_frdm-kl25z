#if !defined(SLEEP_H_)
#define SLEEP_H_   

#include "PeripheralPins.h"
#include "kl25_util.h"

enum SLEEP_EN_MODE{
    SLEEP_VLP_EN=5,
    SLEEP_LLS_EN = 3,
    SLEEP_VLLS_EN=1
};
enum SLEEP_MODE{
    SLEEP_VLPS_MODE=2,
    SLEEP_LLS_MODE = 3,
    SLEEP_VLLS_MODE=4
};




//00 External input pin disabled as wakeup input
//01 External input pin enabled with rising edge detection
//10 External input pin enabled with falling edge detection
// 11 External input pin enabled with any change detection

enum SLEEP_WKUP_PIN_MODE
{
    SLEEP_NO_WAKUP = 0,
    SLEEP_WAKUP_RISING = 1,
    SLEEP_WAKUP_FALLIN = 2,
    SLEEP_WAKUP_CHANGE = 3
};

enum SLEEP_WKUP_SRC
{
    SLEEP_WKUP_FROM_LPTMR = 0,
    SLEEP_WKUP_FROM_CMP = 1,
    SLEEP_WKUP_FROM_TSI = 4,
    SLEEP_WKUP_FROM_RTC_ALARM = 6,
    SLEEP_WKUP_FROM_RTC_SECOND = 7,
};

typedef enum {
    LLWU_P5=5,
    LLWU_P6=6,
   LLWU_P7=7,
    LLWU_P8=8,
    LLWU_P9=9,
    LLWU_P10=10,
     LLWU_P14=14,
    LLWU_P15=15,
} LLUPinName;

const PinMap PinMap_LLWU[] = {
    {PTB0, LLWU_P5,1}, //KO
    {PTC1, LLWU_P6,1}, //KO
    {PTC3, LLWU_P7,1}, //KO
    {PTC4, LLWU_P8,1},  //KO
    {PTC5, LLWU_P9,1},  //KO
    {PTC6, LLWU_P10,1}, //KO
    {PTD4, LLWU_P14,1}, //OK
    {PTD6, LLWU_P15,1}, //OK
    {NC,    NC,0}
};

typedef enum _RESET_SRC {
    SLEEP_RST_WakeUpSource           = 1 << 0,
    SLEEP_RST_LVDSource              = 1 << 1,
    SLEEP_RST_LossOfClockSource      = 1 << 2,
    SLEEP_RST_LossOfLockSource       = 1 << 3,
    SLEEP_RST_WatchDogSource         = 1 << 4,
    SLEEP_RST_ExtPinSource           = 1 << 5,
    SLEEP_RST_PowerOnResetSource     = 1 << 6,
    SLEEP_RST_LockUpSource           = 1 << 7,
    SLEEP_RST_SoftwareSource         = 1 << 8,
    SLEEP_RST_MDM_ApSource           = 1 << 9,
    SLEEP_RST_StopModeAckErrorSource = 1 << 10
} SLEEP_RESET_SRC;


void SMCSleepDeep(SLEEP_MODE mode);

void sleepClearFlag(PinName pin);
int8_t sleepGetFlag(PinName pin);

void sleepLLS(void);
void sleepSrcEnable(SLEEP_WKUP_SRC src);
void sleepSrcDisable(SLEEP_WKUP_SRC src);

void sleepPinEnable(PinName pin, SLEEP_WKUP_PIN_MODE mode);
void sleepInit();

void sleepGoToVLPRunMode(void);
void switchFEItoBLPI( void );
void sleepChangeUart0_9600bps();

#endif