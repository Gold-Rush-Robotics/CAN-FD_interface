#include <Arduino.h>
#include "../knob_servo.h"

#define KNOB_PIN 2

void setup(){
  initKnobServo(KNOB_PIN);
}

void loop(){
  delay(30000);
  spinKnobServo(1000);
  delay(15000);
  spinKnobServo(1500);
}

