# NeoPixelF7

WS2812B Driver for stm32. (Not just F7!)

Supports:
* stm32f7xx: 
  * tested: nucleo_f767zi
  * tested: disco_f746ng
* stm32l4xx
  * tested: nucleo_l432kc
  * tested: nucleo_l476rg
* stm32f3xx
  * tested: nucleo_f303k8
  
Uses Timer PWM with DMA. No buggering about with NOPs and such nonsense.

Hard coded to pin PA8 (GPIOA, GPIO_PIN_8) for now.

Config: NUM_PIXELS for number of LEDs on strip.
Config: TIMER_CLK_FREQ for frequency of timer clock. e.g. On stm32f746g this is AHB2 freq x 2. Which is 216000000 at max speed (such as used by MBED)

Only one dependancy on the stm32 HAL via "stm32XXxx_hal.h".

Tested under MBED 6 (not required) and PlatformIO* (not required). 

Tested under Arduino (via stm32duino) 
REQUIRES HAL_TIM_MODULE_ONLY to be defined...
in build_opts.h for ArduinoIDE (not tested!)
or ```build_flags = -D HAL_TIM_MODULE_ONLY``` for PlatformIO.


PlatformIO: To work as a lib_dep or under the 'lib' folder REQUIRES ```lib_archive = false```
