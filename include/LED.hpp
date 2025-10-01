#ifndef LED_HPP
#define LED_HPP

#include <Arduino.h>

#define LED_LIGHT_RED 5
#define LED_LIGHT_GREEN 4
#define LED_LIGHT_BLUE 3

void led_init();

void led_on(uint8_t r, uint8_t g, uint8_t b);

void led_off();

#endif