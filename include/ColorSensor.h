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
    // other readings
    // (151, 55, 42)
    {
        .name = "Red",
        .match = {255, 0, 0},
        .led = {HIGH, LOW, LOW}
    },
    // other readings
    // (38, 145, 56)
    {
        .name = "Green",
        .match = {0, 255, 0},
        .led = {LOW, HIGH, LOW}
    },
    // other readings
    // (28, 72, 149)
    // (43, 76, 130) perfect
    // (29, 75, 148) left
    // (23, 67, 171)
    {
        .name = "Blue",
        .match = {0, 0, 255},
        .led = {LOW, LOW, HIGH}
    },
    // for now we are ignoring purple and setting the color to purple if it reads blue and the red value is above 35.
    // See code in readThenSetLED.
    // // other readings
    // // (43, 63, 148)
    // {
    //     .name = "Purple",
    //     .match = {43, 63, 255},
    //     .led = {HIGH, LOW, HIGH}
    // }
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

    Color readColor() {
        float r, g, b;
        sensor.getRGB(&r, &g, &b);
        return {(uint8_t) std::round(r), (uint8_t) std::round(g), (uint8_t) std::round(b)};
    }

    void readThenSetLED(size_t led) {
        float r, g, b;
        sensor.getRGB(&r, &g, &b);
        Color read1 = readColor();
        delay(10);
        sensor.getRGB(&r, &g, &b);
        Color read2 = readColor();
        delay(10);
        sensor.getRGB(&r, &g, &b);
        Color read3 = readColor();
        delay(10);
        sensor.getRGB(&r, &g, &b);
        Color read4 = readColor();
        delay(10);
        sensor.getRGB(&r, &g, &b);
        Color read5 = readColor();

        uint8_t red = (read1.r + read2.r + read3.r + read4.r + read5.r)/5;
        uint8_t blue = (read1.b + read2.b + read3.b + read4.b + read5.b)/5;
        uint8_t green = (read1.g + read2.g + read3.g + read4.g + read5.g)/5;
        Color readFinal = {red, green, blue};

        #ifdef COLOR_SENSOR_TUNING
        Serial.print("Color sensor read: ");
        debugColor(readFinal);
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
        if (closestColor.b == HIGH && readFinal.r > 40) {
            closestColor.r = HIGH;
        }

        setLedColor(led, closestColor.r, closestColor.g, closestColor.b);
    }
}
