#include <Arduino.h>
#include <knob_servo.h>
#include <timer.h>

#define KNOB_PIN 2

void setup(){
  initKnobServo(KNOB_PIN);
}

void loop(){
  Timer timer = Timer();

  timer.waitUntil(40000);
  spinKnobServo(1000);
  delay(10000);
  spinKnobServo(1500);

  //stop
  delay(100000000);
}

