#pragma once

#include <Arduino.h>
#include <Adafruit_TCS34725.h>

#include "LED.h"

struct Color {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

struct ReferenceColor {
    const char* name;
    Color match;
    Color led;
};

// Constant colors we reference against.
// We store the name of the color, the RGB to match against, and then the color we should send to our RGB LEDs.
const ReferenceColor COLORS[] = {
    {
        .name = "Red",
        .match = {234, 13, 24},
        .led = {HIGH, LOW, LOW}
    },
    {
        .name = "Green",
        .match = {23, 150, 77},
        .led = {LOW, HIGH, LOW}
    },
    {
        .name = "Blue",
        .match = {10, 50, 200},
        .led = {LOW, LOW, HIGH}
    },
    {
        .name = "Purple",
        .match = {110, 35, 119},
        .led = {HIGH, LOW, HIGH}
    }
};

namespace ColorSensor {
    Adafruit_TCS34725 sensor;

    // Function to calculate the "distance" between the current color and a target color using Euclidean distance in RGB space.
    float calculateEuclidianDistance(Color current, Color match) {
        float diffR = current.r - match.r;
        float diffG = current.g - match.g;
        float diffB = current.b - match.b;
        
        long sumOfSquares = (long)diffR * diffR + (long)diffG * diffG + (long)diffB * diffB;
        
        return sqrt(sumOfSquares);
    }

    void debugColor(Color color) {
        Serial.print("(");
        Serial.print(color.r); Serial.print(", ");
        Serial.print(color.g); Serial.print(", ");
        Serial.print(color.b);
        Serial.println(")");
    }

    uint16_t readLux() {
        float r, g, b;
        sensor.getRGB(&r, &g, &b);
        return sensor.calculateLux(r, g, b);
    }

    void readThenSetLED(size_t led) {
        float r, g, b;
        sensor.getRGB(&r, &g, &b);
        Color read1 = {(uint8_t) std::round(r), (uint8_t) std::round(g), (uint8_t) std::round(b)};
        delay(100);
        sensor.getRGB(&r, &g, &b);
        Color read2 = {(uint8_t) std::round(r), (uint8_t) std::round(g), (uint8_t) std::round(b)};
        delay(100);
        sensor.getRGB(&r, &g, &b);
        Color read3 = {(uint8_t) std::round(r), (uint8_t) std::round(g), (uint8_t) std::round(b)};
        delay(100);
        sensor.getRGB(&r, &g, &b);
        Color read4 = {(uint8_t) std::round(r), (uint8_t) std::round(g), (uint8_t) std::round(b)};
        delay(100);
        sensor.getRGB(&r, &g, &b);
        Color read5 = {(uint8_t) std::round(r), (uint8_t) std::round(g), (uint8_t) std::round(b)};
        uint8_t red = (read1.r + read2.r + read3.r + read4.r + read5.r)/5;
        uint8_t blue = (read1.b + read2.b + read3.b + read4.b + read5.b)/5;
        uint8_t green = (read1.g + read2.g + read3.g + read4.g + read5.g)/5;
        Color readFinal = {red, green, blue};

        #ifdef COLOR_SENSOR_DEBUG
        Serial.print("Color sensor read: ");
        debugColor(read);
        #endif

        float lowestDistance = INFINITY;
        Color closestColor = {LOW, LOW, LOW};
        for (ReferenceColor color : COLORS) {
            float distance = calculateEuclidianDistance(readFinal, color.match);
            if (distance < lowestDistance) {
                lowestDistance = distance;
                closestColor = color.led;
            }

            #ifdef COLOR_SENSOR_DEBUG
            Serial.print("Distance from ");
            Serial.print(color.name);
            Serial.print(": ");
            Serial.println(distance);
            #endif
        }

        setLedColor(led, closestColor.r, closestColor.g, closestColor.b);
    }
}
