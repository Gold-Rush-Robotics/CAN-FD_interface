#pragma once

#include <Arduino.h>
#include <cinttypes>

enum Message: uint8_t {
    Invalid = 0,
    Ping = 1,
    Pong = 2,
    StartGame = 3,
    MoveArm = 4,
    MoveBugs = 5,
    ReadThenSetLED = 6
};
const uint8_t MESSAGE_MAX = Message::ReadThenSetLED;

namespace SerialAtomics {
    #define Serial Serial7

    void setup() {
        Serial.begin(9600);
    }

    void send(uint8_t byte) {
        Serial.write(byte);
    }

    uint8_t recvByte() {
        while (!Serial.available()) {
            delay(10);
        }

        return (uint8_t) Serial.read();
    }

    #undef Serial

    Message recvMsg() {
        uint8_t byte = recvByte();
        if (byte <= MESSAGE_MAX) {
            return (Message) byte;
        } else {
            Serial.println("Error: Received undefined message");
            return Message::Invalid;
        }
    }
}