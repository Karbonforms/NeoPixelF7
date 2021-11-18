#ifndef MIDILIGHTSF7_NEOPIXELF7_H
#define MIDILIGHTSF7_NEOPIXELF7_H


#define RESET_PADDING 50
#if defined(TARGET_STM32F7)
#define STM32F7xx
#elif defined(TARGET_STM32L4)
#define STM32L4xx
#elif defined(TARGET_STM32F3)
#define STM32F3xx
#elif defined(TARGET_STM32F1)
#define STM32F1xx
#elif defined(TARGET_STM32F0)
#define STM32F0xx
#elif defined(TARGET_STM32G0)
#define STM32G0xx
#elif defined(TARGET_STM32G4)
#define STM32G4xx
#endif


#if defined(STM32L4xx)
#   include <stm32l4xx_hal.h>
#elif defined(STM32F7xx)
#   include <stm32f7xx_hal.h>
#elif defined(STM32F3xx)
#   include <stm32f3xx_hal.h>
#elif defined(STM32G0xx)
#   include <stm32g0xx_hal.h>
#elif defined(STM32G4xx)
#   include <stm32g4xx_hal.h>
#elif defined(STM32F1xx)
#   include <stm32f1xx_hal.h>
#elif defined(STM32F0xx)
#   include <stm32f0xx_hal.h>
#endif

//// These assume running at full speed. (Which is the case under Arduino and MBed).
//#ifndef TIMER_CLK_FREQ
//#   if defined(STM32F7xx)
//#       define TIMER_CLK_FREQ (216000000u)
//#   elif defined(STM32L4xx)
//#       define TIMER_CLK_FREQ (80000000u)
//#   elif defined(STM32F3xx) || defined(STM32F103xB)
//#       define TIMER_CLK_FREQ (72000000u)
//#   elif defined(STM32F3xx) || defined(STM32F103xB)
//#       define TIMER_CLK_FREQ (72000000u)
//#   elif defined(STM32G0xx)
//#       define TIMER_CLK_FREQ (64000000u)
//#   elif defined(STM32G4xx)
//#       define TIMER_CLK_FREQ (170000000u)
//#   endif
//#endif



// On PlatformIO you can use "build_flags = -D NUM_PIXELS=(n)"
#ifndef NUM_PIXELS
#define NUM_PIXELS     (1)
#endif



void NeoPixelF7_init();
void NeoPixelF7_show(const uint32_t* ptr, uint32_t num_pixels);

class Pixels
{
public:
    explicit Pixels(uint32_t* arr, uint32_t len);
    static void begin();
    static uint32_t create_color(const uint8_t& red, const uint8_t& green, const uint8_t& blue);
    void set_color(const uint32_t& index, const uint32_t& color );
    void set_rgb(const uint32_t& index, const uint8_t& red, const uint8_t& green, const uint8_t& blue);
    void clear(const uint32_t& index);
    void clear_all();
    void show();
private:
    uint32_t* pixels_;
    uint32_t  len_;
};

#endif //MIDILIGHTSF7_NEOPIXELF7_H
