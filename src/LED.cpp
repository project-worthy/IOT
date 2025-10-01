#include "LED.hpp"

uint8_t led_power = 0;

void led_init()
{
    pinMode(LED_LIGHT_RED, OUTPUT);
    pinMode(LED_LIGHT_GREEN, OUTPUT);
    pinMode(LED_LIGHT_BLUE, OUTPUT);
}

void led_on(uint8_t r, uint8_t g, uint8_t b)
{
    analogWrite(LED_LIGHT_RED, r);
    analogWrite(LED_LIGHT_GREEN, g);
    analogWrite(LED_LIGHT_BLUE, b);
}

void led_off()
{
    analogWrite(LED_LIGHT_RED, 0);
    analogWrite(LED_LIGHT_GREEN, 0);
    analogWrite(LED_LIGHT_BLUE, 0);
}