#pragma once

#include <Arduino.h>
#include <SCServo.h>

struct ArmPosition {
    int servo1;
    int servo2;
};

// Predefined arm positions.
namespace ArmPositions {
    const ArmPosition READ_COLOR = {1000, 1000};
    const ArmPosition COLLAPSED = {0, 2000};
};

namespace Arm {
    SMS_STS controller = SMS_STS();

    void move(ArmPosition pos) {
        if (
            pos.servo1 < 0 ||
            pos.servo1 > 2000 ||
            pos.servo2 < 0 ||
            pos.servo2 > 2000
        ) {
            Serial.println("Ignoring moveArm command; servos can only move between 0 and 2000.");
            return;
        }

        controller.WritePosEx(1, pos.servo1, 2400, 50);
        controller.WritePosEx(2, pos.servo2, 2400, 50);

        delay(1000);
        
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