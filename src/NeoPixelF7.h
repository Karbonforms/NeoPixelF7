#ifndef MIDILIGHTSF7_NEOPIXELF7_H
#define MIDILIGHTSF7_NEOPIXELF7_H

#if defined(STM32F7xx)
#define TIMER_CLK_FREQ (216000000)
#elif defined(STM32L4xx)
#define TIMER_CLK_FREQ (80000000)
#elif defined(STM32F3xx)
#define TIMER_CLK_FREQ (72000000)
#endif

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
    static uint32_t create_color(uint8_t red, uint8_t green, uint8_t blue);
    void set_color(uint32_t index, uint32_t color );
    void set_rgb(uint32_t index, uint8_t red, uint8_t green, uint8_t blue);
    void clear_rgb(uint32_t index);
    void show();
private:
    uint32_t* pixels_;
    uint32_t  len_;
};

#endif //MIDILIGHTSF7_NEOPIXELF7_H
