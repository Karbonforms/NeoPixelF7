# NeoPixelSTM32

Update 1.1.0: Timer clock speed is now calculated.

WS2812B Driver for STM32
Uses Timer PWM with DMA. No buggering about with NOPs and such nonsense.

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
* stm32f1xx UNTESTED (blue/black pills)



Theoretical max framerate of 144 WS2812Bs is 231.48fps. This lib's doing 228fps (on nucleo_l432kc).

Hard coded to pin PA8 (GPIOA, GPIO_PIN_8) for now.

Config: NUM_PIXELS must be defined for number of LEDs on strip. See example.

Only one dependency on the stm32 HAL via "stm32XXxx_hal.h". (included with Arduino and MBed)

Tested under MBED 6 (PlatformIO), and Arduino (via PlatformIO/stm32duino).

TODO:
* Multi-strip
* current limiting
* color defines?
