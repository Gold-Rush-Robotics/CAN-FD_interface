#include "motor_controller.h"

MotorController::MotorController(int dirPin, int pwmPin, int slpPin, int fltPin,
                                 int encA, int encB, int csPin)
    : _dirPin(dirPin), _pwmPin(pwmPin), _slpPin(slpPin),
      _fltPin(fltPin), _encA(encA), _encB(encB), _csPin(csPin) {
  _encoder = new Encoder(_encA, _encB);
}

void MotorController::begin() {
  pinMode(_dirPin, OUTPUT);
  pinMode(_pwmPin, OUTPUT);
  pinMode(_slpPin, OUTPUT);
  pinMode(_fltPin, INPUT);
  pinMode(_csPin, INPUT);

  digitalWrite(_slpPin, HIGH); // enable
}

void MotorController::setSpeed(int pwmVal) {
  pwmVal = constrain(pwmVal, -255, 255);
  digitalWrite(_slpPin, HIGH);
  if (pwmVal >= 0) digitalWrite(_dirPin, HIGH);
  else digitalWrite(_dirPin, LOW);

  analogWrite(_pwmPin, abs(pwmVal));
}

void MotorController::setSpeedRPM(float rpm) {
  // Simple mapping — in real code you’d have PID control
  int pwm = map((int)rpm, -100, 100, -255, 255);
  setSpeed(pwm);
}

float MotorController::getRPM() {
  unsigned long now = millis();
  long encCount = _encoder->read();
  long delta = encCount - _lastEncoderCount;
  float revs = delta / (float)(_ticksPerRev * _gearRatio);
  float dt = (now - _lastTime) / 60000.0; // min
  float rpm = (dt > 0) ? (revs / dt) : 0;

  _lastEncoderCount = encCount;
  _lastTime = now;
  return rpm;
}
