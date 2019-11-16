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
-  main with a lot of tests inside (IIR FIR filter, test ADC DAC,...). A lots of programs are in cpp drivers and have just to be copied to main.cpp function.
### How to use
- Install [VS code](https://code.visualstudio.com/)
- add plugin **Platformio** in VS Code
- create a new project from Home with FRDM-KL25z board and mbed framework
- copy all files in src directory inside src directory in your folder inside platformio.
- enjoy