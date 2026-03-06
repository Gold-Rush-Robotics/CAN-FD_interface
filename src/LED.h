#pragma once

#include <Arduino.h>

// LED pin definitions. Each LED (4 in total) has 3 pins for R, G, and B respectively.
const int LEDS[][3] = {
    {13, 14, 15},
    {41, 40, 39},
    {38, 37, 36},
    {35, 34, 33}
};

// Function to set the color of a specific LED (0-3) using a Color struct.
void setLedColor(int led, bool r, bool g, bool b) {
    if (led >= std::size(LEDS) || led < 0) {
        Serial.println("Invalid LED index");
        return;
    }

    digitalWrite(LEDS[led][0], r);
    digitalWrite(LEDS[led][1], g);
    digitalWrite(LEDS[led][2], b);  
}