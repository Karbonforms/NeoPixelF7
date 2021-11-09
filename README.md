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
* stm32g0xx UNTESTED 
* stm32g4xx UNTESTED 
  
Uses Timer PWM with DMA. No buggering about with NOPs and such nonsense.

Theoretical max framerate of 144 WS2812Bs is 231.48fps. This lib's doing 228fps (on nucleo_l432kc).

Hard coded to pin PA8 (GPIOA, GPIO_PIN_8) for now.

Config: NUM_PIXELS for number of LEDs on strip.
Config: TIMER_CLK_FREQ for frequency of timer clock. e.g. On stm32f746g this is AHB2 freq x 2. Which is 216000000 at max speed (such as used by MBED)

Only one dependency on the stm32 HAL via "stm32XXxx_hal.h". (included with Arduino and MBed)

Tested under MBED 6 (not required) and PlatformIO* (not required). 

Tested under Arduino (via stm32duino).

TODO:
* Multi-strip
* current limiting