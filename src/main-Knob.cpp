#include <Arduino.h>
#include <knob_servo.h>
#include <timer.h>

#define KNOB_PIN 2

void setup(){
  initKnobServo(KNOB_PIN);
}

void loop(){
  delay(35000);

  
  spinKnobServo(1000);
  delay(30000);
  spinKnobServo(1500);

  //stop
  delay(100000000);
}

