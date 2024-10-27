#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>

#define ANCHOR 0
#define TAG 1

#define MODULE_ID 4 // change number to set anchor

#if MODULE_ID == 1
static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'S', 'I', 'J', 'A', 0xE0, 0, 0};
static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'J', 'A', 'S', 'I', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
#elif MODULE_ID == 2
static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
#elif MODULE_ID == 3
/* Frames used in the ranging process. See NOTE 3 below. */
static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'D', 'S', 'B', 'C', 0xE0, 0, 0};
static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'B', 'C', 'D', 'S', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
#elif MODULE_ID == 4
static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'K', 'I', 'R', 'A', 0xE0, 0, 0};
static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'R', 'A', 'K', 'I', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
#endif

static double preset_delay[3] = {16350, 16350, 16300};
extern double preset_delay[3];

void print_INFO(const char *name, const char *message);

void print_WARN(const char *name, const char *message);

#endif