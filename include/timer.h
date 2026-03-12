#pragma once
#include <Arduino.h>

class Timer {
    public:
    unsigned long startTime;

    Timer() {
        startTime = millis();
    }

    void waitUntil(unsigned long endTime) {
        while(millis() - startTime < endTime) { 
            //spinlock
        }
    }

    void waitFor(unsigned long ms) {
        waitUntil(millis() + ms);
    }
};