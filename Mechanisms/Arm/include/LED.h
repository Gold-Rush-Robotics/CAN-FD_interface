#pragma once

#include <Arduino.h>

// LED pin definitions. Each LED (4 in total) has 3 pins for R, G, and B respectively.
const int LEDS[][3] = {
    {21, 17, 16},
    {2, 3, 4},
    {41, 13, 40},
    {7, 8, 9}
};

// Function to set the color of a specific LED (0-3) using a Color struct.
void setLedColor(size_t led, bool r, bool g, bool b) {
    if (led >= std::size(LEDS) || led < 0) {
        Serial.println("Invalid LED index");
        return;
    }

    digitalWrite(LEDS[led][0], r);
    digitalWrite(LEDS[led][1], g);
    digitalWrite(LEDS[led][2], b);  
}


void testAllLeds() {
    for(int i = 0; i < 4; i++) {
        setLedColor(i, 1, 0, 0);
        delay(1000);
        setLedColor(i, 0, 1, 0);
        delay(1000);
        setLedColor(i, 0, 0, 1);
        delay(1000);
        setLedColor(i, 1, 0, 1);
        delay(1000);

        setLedColor(i, 0, 0, 0);
    }
}