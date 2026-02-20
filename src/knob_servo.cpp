#include "knob_servo.h"

static Servo knobServo;
static bool isAttached = false;

void initKnobServo(int servoPin){
    if (!isAttached){
        knobServo.attach(servoPin);
        isAttached = true;
    }
}

void spinKnobServo(int pulse){
    knobServo.writeMicroseconds(pulse);
}


