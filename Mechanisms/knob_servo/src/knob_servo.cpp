#include "knob_servo.h"

static Servo knobServo;
static bool isAttached = false;

void initKnobServo(int servoPin){
    if (!isAttached){
        knobServo.attach(servoPin);
        isAttached = true;
    }
}

// Counter Clockwise < 1500 < Clockwise | Closer to 1500 = slower
void spinKnobServo(int pulse){
    knobServo.writeMicroseconds(pulse);
}


