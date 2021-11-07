#include <Arduino.h>
#define
#include "NeoPixelF7.h"

#define BRIGHTNESS 40

// create array to hold pixel colors
uint32_t LEDS[144];

// create wrapper
Pixels pixels{LEDS, 144};

void setup() {
    Serial.begin(9600);
    pixels.begin(); // or NeoPixelF7_init();
}

void do_pixel(uint32_t index, unsigned long color)
{
    pixels.set_color(index, color);
    pixels.show();
    pixels.clear_rgb(index);
}

void loop() {
    Serial.println("LOOP");
    static uint32_t red = Pixels::create_color(BRIGHTNESS,0,0);
    static uint32_t grn = Pixels::create_color(0,BRIGHTNESS,0);
    static uint32_t blu = Pixels::create_color(0,0,BRIGHTNESS);
    static uint32_t colors[] = {red, grn, blu};
    for (unsigned long color : colors)
    {
        for (uint32_t i = 0; i < 144; ++i)
        {
            do_pixel(i, color);
        }
        for (int32_t i = 143; i >= 0; --i)
        {
            do_pixel(i, color);
        }
    }
}