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
    void setup() {
        Serial7.begin(9600);
    }

    void send(uint8_t byte) {
        Serial7.write(byte);
    }

    uint8_t recvByte() {
        while (Serial7.available() < 1) {
            delay(10);
        }
        int byte = Serial7.read();

        return (uint8_t) byte;
    }

    Message recvMsg() {
        uint8_t byte = recvByte();
        if (byte <= MESSAGE_MAX && byte >= 0) {
            return (Message) byte;
        } else {
            Serial.println("Error: Received undefined message " + String(byte));
            return Message::Invalid;
        }
    }
}
