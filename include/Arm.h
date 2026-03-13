#pragma once

#include <Arduino.h>
#include <SCServo.h>

#define SPEED 2400
#define ACCELERATION 50

struct ArmPosition {
    int servo1;
    int servo2;
};

// Predefined arm positions.
const ArmPosition _ARM_POSITIONS[] = {
    {53, 128},
    {10, 180},
    {173, 128},
    {173, 50},
};
// Indices for the predefined arm positions. This lets the teensies communicate
// an arm position over serial.
namespace ArmPositions {
    const uint8_t READ_COLOR = 0;
    const uint8_t COLLAPSED = 1;
    const uint8_t DOWN = 2;
    const uint8_t KNOCK_DUCK = 3;
};

namespace BugPositions {
    const int COLLAPSED = 180;
    const int HELLDIVE = 30;
}

namespace Servos {
    SMS_STS controller = SMS_STS();

    void move(int id, int pos) {
        pos = map(pos, 0, 180, 0, 2000);

        if (
            pos < 0 ||
            pos > 2000
        ) {
            Serial.println("Ignoring move command; servos can only move between 0 and 2000.");
            return;
        }

        controller.WritePosEx(id, pos, SPEED * 1.25, ACCELERATION * 1.25);
    }

    void moveArm(uint8_t pos_idx) {
        if (pos_idx >= std::size(_ARM_POSITIONS)) {
            Serial.println("ERROR: Tried to move to nonexistant arm position");
            return;
        }

        ArmPosition pos = _ARM_POSITIONS[pos_idx];
        move(1, pos.servo1);
        move(2, pos.servo2);
        
        #ifdef ARM_DEBUG
        for (int i = 1; i <= 2; i++) {
            Serial.print("Servo ");
            Serial.print(i);
            Serial.print("'s position: ");
            Serial.println(controller.ReadPos(i));
        }
        Serial.println();
        #endif
    }
};
