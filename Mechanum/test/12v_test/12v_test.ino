#include <Servo.h>

Servo esc;

const int ESC_PIN = 2;          // Any digital pin works on Teensy 4.1
const int MIN_THROTTLE = 1000;  // microseconds
const int MAX_THROTTLE = 2000;

void setup() {
  Serial.begin(115200);
  delay(2000);

  esc.attach(ESC_PIN, MIN_THROTTLE, MAX_THROTTLE);

  Serial.println("Arming ESC...");
  esc.writeMicroseconds(MIN_THROTTLE);
  delay(3000);  // give ESC time to arm
}

void loop() {
  // Ramp up
  for (int pwm = MIN_THROTTLE; pwm <= MAX_THROTTLE; pwm += 10) {
    esc.writeMicroseconds(pwm);
    delay(20);
  }

  delay(2000); // hold full throttle briefly

  // Ramp down
  for (int pwm = MAX_THROTTLE; pwm >= MIN_THROTTLE; pwm -= 10) {
    esc.writeMicroseconds(pwm);
    delay(20);
  }

  delay(3000);
}
