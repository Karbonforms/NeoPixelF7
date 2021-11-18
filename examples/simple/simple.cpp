#include <Arduino.h>
#include "NeoPixelF7.h"

#define BRIGHTNESS 20

extern uint32_t g_DataWaiting;

// create array to hold pixel colors
uint32_t LEDS_array[NUM_PIXELS];

// create wrapper
Pixels pixels{LEDS_array, NUM_PIXELS};

void do_pixel(uint32_t index, uint32_t color)
{
    pixels.set_color(index, color);
    pixels.show();
    pixels.clear(index);
}

void setup()
{
    Serial.begin(9600);
    Pixels::begin(); // or NeoPixelF7_init();
}

void loop()
{
    static uint32_t colors[] = {
            Pixels::create_color(BRIGHTNESS, 0, 0),
            Pixels::create_color(0, BRIGHTNESS, 0),
            Pixels::create_color(0, 0, BRIGHTNESS)
    };
    uint32_t frame_count = 0;
    uint32_t start = micros();
    for (uint32_t color : colors)
    {
        for (uint32_t i = 0; i < NUM_PIXELS; ++i)
        {
            do_pixel(i, color);
            frame_count++;
        }
        for (int32_t i = (NUM_PIXELS-1); i >= 0; --i)
        {
            do_pixel(i, color);
            frame_count++;
        }
    }
    const uint32_t elapsed = micros() - start;
    const uint32_t us_per_frame = elapsed / frame_count;
    const uint32_t fps = 1000000 / us_per_frame;
    Serial.printf("frames: %d\telapsed: %d\t fps: %d\r\n", frame_count, elapsed, fps);
    Serial.printf("data waiting: %dms\r\n", g_DataWaiting);
}