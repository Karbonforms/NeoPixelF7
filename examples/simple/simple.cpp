#include <Arduino.h>
#include "NeoPixelF7.h"

#define BRIGHTNESS 20

// create array to hold pixel colors
uint32_t LEDS_array[NUM_PIXELS];

// create wrapper
Pixels pixels{LEDS_array, NUM_PIXELS};

void setup()
{
    Pixels::begin(); // or NeoPixelF7_init();
}

void do_pixel(uint32_t index, uint32_t color)
{
    pixels.set_color(index, color);
    pixels.show();
    pixels.clear_rgb(index);
}

void loop()
{
    static uint32_t colors[] = {
            Pixels::create_color(BRIGHTNESS, 0, 0),
            Pixels::create_color(0, BRIGHTNESS, 0),
            Pixels::create_color(0, 0, BRIGHTNESS)
    };
    for (unsigned long color : colors)
    {
        for (uint32_t i = 0; i < NUM_PIXELS; ++i)
        {
            do_pixel(i, color);
        }
        for (int32_t i = (NUM_PIXELS-1); i >= 0; --i)
        {
            do_pixel(i, color);
        }
    }
}