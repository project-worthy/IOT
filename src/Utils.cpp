#include "Utils.h"

#include <Arduino.h>

void print_INFO(const char* name, const char* message) {
    Serial.println("[INFO] - " + String(name) + " : " + String(message));
}

void print_WARN(const char* name, const char* message) {
    Serial.println("[WARN] - " + String(name) + ": message");
}