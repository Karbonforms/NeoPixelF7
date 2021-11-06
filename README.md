# NeoPixelF7

WS2812B Driver for (initially) stm32f746g. Others coming soon...

Uses Timer PWM with DMA. No buggering about with NOPs and such nonsense.

Hard coded to pin D10 (GPIOA, GPIO_PIN_8) for now.

Config: NUM_PIXELS for number of LEDs on strip.
Config: TIMER_CLK_FREQ for frequency of timer clock. On stm32f746g this is AHB2 freq x 2. Which is 216000000 at max speed (such as used by MBED)

Tested under MBED 6 (not required) and PlatformIO* (not required). Only one dependancy on the stm32 HAL via "stm32f7xx_hal.h".


* Does not (yet) work as a lib_dep or under the 'lib' folder. Something to do with linking and the HAL .weak Callbacks. I'll figure it out...
