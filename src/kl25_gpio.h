#if !defined(GPIO_H_)
#define GPIO_H_

#include "mbed.h"
#include "kl25_util.h"

/************ pin GPIO ***********
 * D2 = PTD4
 * D3 = PA12
 * D4 = PTA4
 * D5 = PTA5
 * D6 = PTC8
 * D7 = PTC9
 * D8 = PTA13
 * D9 = PTD5
 * D10 = PTD0
 * D11 = PTD2
 * D12 = PTD3
 * D13 = PTD1
 * ******************************/

enum Mode
{
    GPIO_MODE_PullUp=0,
    GPIO_MODE_PullDown=1,
    GPIO_MODE_PullNone=2
};

enum ModeInterrupt
{
    GPIO_MODE_INTERRUPT_LOW = 0b1000,
    GPIO_MODE_INTERRUPT_RISING = 0b1001,
    GPIO_MODE_INTERRUPT_FALLING = 0b1010,
    GPIO_MODE_INTERRUPT_CHANGE = 0b1011,
    GPIO_MODE_INTERRUPT_HIGH = 0b1100
};
//High level function
void gpioDigitalOut(PinName pin, int value);
void gpioDigitalIn(PinName pin);
//interrupt functions
void gpioPortAEnableInterrupt(int pin, ModeInterrupt mode);
void gpioPortADisableInterrupt(int pin);
void gpioPortA_attachInterrupt(void (*userFunc)(void));
void gpioPort_attachPORTA_IRQHandler();
void gpioPortA_dettachInterrupt();
void gpioPortDEnableInterrupt(int pin, ModeInterrupt mode);
void gpioPortDDisableInterrupt(int pin);
void gpioPortD_attachInterrupt(void (*userFunc)(void));
void gpioPort_attachPORTD_IRQHandler();
void gpioPortD_dettachInterrupt();
//inline function for IRQ Handler 
inline void gpioClearInterrupt(PORT_Type *port, int pin)
{
     port->ISFR = (1 << pin);
}
inline int gpioIsInterruptPending(PORT_Type *port, int pin)
{
     return (port->ISFR & (1 << pin));
}
//Low level functions
void gpioEnableCLK(PortName port);
void gpioInitOut(FGPIO_Type *fast_gpio_port, int pin, bool value);
void gpioInitIn(FGPIO_Type *fast_gpio_port, int pin);
void gpioMode(PORT_Type *gpio_port, int pin, Mode mode);
void gpioEnablePin(PORT_Type *gpio_port, int pin);
//fast function for one cycle register FPTi
inline void fgpioSet(FGPIO_Type *fast_gpio_port, int pin)
{
    fast_gpio_port->PSOR |= (1 << pin);
}
inline void fgpioReset(FGPIO_Type *fast_gpio_port, int pin)
{
    fast_gpio_port->PCOR |= (1 << pin);
}
inline void fgpioToggle(FGPIO_Type *fast_gpio_port, int pin)
{
    fast_gpio_port->PTOR |= (1 << pin);
}
inline int fgpioRead(FGPIO_Type *fast_gpio_port, int pin)
{
    return ((fast_gpio_port->PDIR & (1 << pin)) == (uint32_t)(1 << pin));
}
//fast functions for multi cycles write or read register PORTi
inline void gpioSet(GPIO_Type *gpio_port, int pin)
{
    gpio_port->PSOR |= (1 << pin);
}
inline void gpioReset(GPIO_Type *gpio_port, int pin)
{
    gpio_port->PCOR |= (1 << pin);
}
inline void gpioToggle(GPIO_Type *gpio_port, int pin)
{
    gpio_port->PTOR |= (1 << pin);
}
inline int gpioRead(GPIO_Type *gpio_port, int pin)
{
    return ((gpio_port->PDIR & (1 << pin)) == (uint32_t)(1 << pin));
}

/* tested but too long for IRQ Handler -> abandonned
//Dichotomy search as in gpio_irq_api.c
const uint32_t search_bits[] = {0x0000FFFF, 0x000000FF, 0x0000000F, 0x00000003, 0x00000001};
inline uint8_t gpioSearchPinLocation(PORT_Type *port_n)
{
    //search whish pins was activate (dichotomy method)
    uint32_t isfr;
    uint8_t location=0;
    if ((isfr = port_n->ISFR) != 0)
    {
        location = 0;
        for (int i = 0; i < 5; i++)
        {
            if (!(isfr & (search_bits[i] << location)))
                location += 1 << (4 - i);
        }    
//        printf("location=%d\n\r",location);
    }
    return location;
    
}
//search the last pin enabled for interrupt
inline int gpioSearchPinEnabled(PORT_Type *port_n)
{
    int i;
    for (i = 0; i < 32; i++) //can be optimized, no 32 pins enabled
    {
        if ((port_n->PCR[i] & PORT_PCR_IRQC_MASK) != 0)
            return i;
    }
}*/
#endif