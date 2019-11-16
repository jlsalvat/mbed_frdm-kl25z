#if !defined(UTIL_H_)
#define UTIL_H_   
#include "mbed.h" 
#include "PeripheralPins.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MCG_S_IRCST_VAL ((MCG->S & MCG_S_IRCST_MASK) >> MCG_S_IRCST_SHIFT)
#define MCG_S_CLKST_VAL ((MCG->S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT)
#define MCG_S_IREFST_VAL ((MCG->S & MCG_S_IREFST_MASK) >> MCG_S_IREFST_SHIFT)

enum CLK_INTERNAL_SEL{
    CLK_INTERNAL_32kHZ=0,
    CLK_INTERNAL_4MHz=1
};
//MAX Period if clk=48MHz => T=128/48000000*65536=174ms
enum TPM_Prescaler{
    TPM_DIV_1 = 0,
    TPM_DIV_2 = 1,
    TPM_DIV_4 = 2,
    TPM_DIV_8 = 3,
    TPM_DIV_16 = 4,
    TPM_DIV_32 = 5,
    TPM_DIV_64 = 6,
    TPM_DIV_128 = 7
};

enum TPM_Clk_Sel{
    TPM_NO_CLK = 0,
    TPM_CLK_EXT_PIN=0x02,
    TPM_CLK_PLL_48MHz=0x11,
//    TPM_CLK_EXT_8MHz=0x21, //32kHz ??? pb 
    TPM_CLK_INTERNAL_4MHz=0x131, // 4Mhz   
    TPM_CLK_INTERNAL_32kHz=0x31 // 32kHz
};

typedef struct {
    unsigned int port;
    unsigned int tpm_n;
    unsigned int ch_n;
    unsigned int pin_n;
    unsigned int alternate;
} PortInfo;

/* Macro to enable all interrupts. */
#define EnableInterrupts asm ("CPSIE  i")
 
/* Macro to disable all interrupts. */
#define DisableInterrupts asm ("CPSID  i")

#define BIT_SET(x) (1UL << (x))
#define BIT_RESET(x) ~(1UL << (x))

void clkInternalSelection(int clk_internal);
PortInfo findPortInfo(PinName pin);
void tpmExternalClockSelection(PortName port, int pin);

#ifdef __cplusplus
}
#endif

#endif