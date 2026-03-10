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
namespace ArmPositions {
    const ArmPosition READ_COLOR = {43, 128};
    const ArmPosition COLLAPSED = {0, 180};
};

namespace BugPositions {
    const int COLLAPSED = 0;
    const int HELLDIVE = 120;
}

namespace Servos {
    SMS_STS controller = SMS_STS();

    void move(int id, int pos) {
        pos = map(pos, 0, 180, 0, 2000);

        if (
            pos < 0 ||
            pos > 2000
        ) {
            Serial.println("Ignoring moveArm command; servos can only move between 0 and 2000.");
            return;
        }

        controller.WritePosEx(id, pos, SPEED, ACCELERATION);
    }

    void moveArm(ArmPosition pos) {
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