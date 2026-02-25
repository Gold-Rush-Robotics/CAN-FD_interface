#include "motor_controller.h"

MotorController::MotorController(int dirPin, int pwmPin, int slpPin, int fltPin, int encA, int encB, int csPin, int starting_direction)
    : _dirPin(dirPin), _pwmPin(pwmPin), _slpPin(slpPin), _fltPin(fltPin), _encA(encA), _encB(encB), _csPin(csPin), starting_direction(starting_direction) {
  _encoder = new Encoder(_encA, _encB);
  if (!_encoder) {
    Serial.println("ERROR: Failed to allocate Encoder");
  }
}

bool MotorController::begin() {
  if (!_encoder) {
    Serial.println("ERROR: Encoder not initialized");
    return false;
  }
  pinMode(_dirPin, OUTPUT);
  pinMode(_pwmPin, OUTPUT);
  pinMode(_slpPin, OUTPUT);
  pinMode(_fltPin, INPUT);
  pinMode(_csPin, INPUT);
  digitalWrite(_slpPin, HIGH);
  return true;
}

void MotorController::setSpeed(int pwmVal) {
  pwmVal = constrain(pwmVal, -255, 255);
  digitalWrite(_slpPin, HIGH);
  digitalWrite(_dirPin, pwmVal*starting_direction >= 0 ? HIGH : LOW);
  analogWrite(_pwmPin, abs(pwmVal));
}

void MotorController::setSpeedRPM(float rpm) {
  int pwm = map(rpm, -100, 100, -255, 255);
  setSpeed(pwm);
}


float MotorController::getRPM() {
  if (!_encoder) {
    Serial.println("ERROR: Encoder not available");
    return 0.0f;
  }
  unsigned long now = millis();
  long encCount = _encoder->read();
  long delta = encCount - _lastEncoderCount;
  float revs = delta / (float)(_ticksPerRev * _gearRatio);
  float dt = (now - _lastTime) / 60000.0;
  float rpm = (dt > 0) ? (revs / dt) : 0;
  _lastEncoderCount = encCount;
  _lastTime = now;
  return rpm;
}

int MotorController::getFaultPin() {
  return _fltPin;
}