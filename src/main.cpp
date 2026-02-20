#include <Arduino.h>
#include "knob_servo.h"

#define KNOB_PIN 2

void setup(){
  initKnobServo(KNOB_PIN);
}

void loop(){
  delay(5000);
  spinKnobServo(2000);
  delay(5000);
}

