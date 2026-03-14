#include <Arduino.h>
#include <knob_servo.h>
#include <timer.h>

#define KNOB_PIN 2
#define ARM_CONNECTION 6

void setup(){
  initKnobServo(KNOB_PIN);
  pinMode(ARM_CONNECTION, INPUT);
  spinKnobServo(1500);
}

void loop(){
  if (digitalRead(ARM_CONNECTION)) {
    spinKnobServo(1000);
  } else {
    spinKnobServo(1500);
  }
}

