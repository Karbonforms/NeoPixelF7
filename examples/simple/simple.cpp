#include <Arduino.h>

#include "src/NeoPixelF7.h"

#define BRIGHTNESS 40
uint32_t LEDS[144];

uint32_t create_color(uint8_t red,uint8_t green,uint8_t blue )
{
    return (green << 16 | red << 8 | blue);
}

void set_color(uint32_t index, uint32_t color )
{
    LEDS[index] = color;
}

void set_rgb(uint32_t index, uint8_t red, uint8_t green, uint8_t blue)
{
    LEDS[index] = (green << 16 | red << 8 | blue);
}

void clear_rgb(uint32_t index)
{
    LEDS[index] = 0;
}

void setup() {
    Serial.begin(9600);
    NeoPixelF7_init();
}

void do_pixel(unsigned long color, uint32_t i)
{
    set_color(i, color);
    NeoPixelF7_show(LEDS, 144);
    clear_rgb(i);
}

void loop() {
    Serial.println("LOOP");
    static uint32_t red = create_color(BRIGHTNESS,0,0);
    static uint32_t grn = create_color(0,BRIGHTNESS,0);
    static uint32_t blu = create_color(0,0,BRIGHTNESS);
    static uint32_t colors[] = {red, grn, blu};
    for (unsigned long color : colors)
    {
        for (uint32_t i = 0; i < 144; ++i)
        {
            do_pixel(color, i);
        }
        for (int32_t i = 143; i >= 0; --i)
        {
            do_pixel(color, i);
        }
    }
}