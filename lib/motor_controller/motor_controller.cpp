#include <motor_controller.h>
#include <cmath>

MotorController::MotorController(int dirPin, int pwmPin, int slpPin, int fltPin, int encA, int encB, int csPin, int starting_direction, double Kp, double Ki, double Kd, double POn)
    : _dirPin(dirPin), _pwmPin(pwmPin), _slpPin(slpPin), _fltPin(fltPin), _encA(encA), _encB(encB), _csPin(csPin), starting_direction(starting_direction) {
  _encoder = new Encoder(_encA, _encB);

  _pid = new PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, POn, DIRECT);
  _pid->SetMode(AUTOMATIC);
  _pid->SetOutputLimits(50,-50);
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
  Setpoint = (double)rpm;
  int pwm = map((int)rpm, 100, -100, 255, -255);
  setSpeed(pwm);
}

void MotorController::PidSetSpeedRPM(float rpm) {
  int pwm = map((int)rpm, 100, -100, 255, -255);
  setSpeed(pwm);
}


float MotorController::getRPM() {
  if (!_encoder) {
    Serial.println("ERROR: Encoder not available");
    return 0.0f;
  }
  unsigned long now = millis();
  float timeElapsed = (now - _lastTime);
  if (timeElapsed > 50) {
      long encCount = _encoder->read();
      long delta = (encCount - _lastEncoderCount) * -1;

      float revs = delta / (float)(_ticksPerRev * _gearRatio);
      float dt = timeElapsed / 60000.0;
      float rpm = revs / dt;
      _lastEncoderCount = encCount;
      _lastTime = now;
      _lastRPM = rpm;
  }
  return _lastRPM;
}

void MotorController::PidLoop() {
  Input = (double)getRPM();
  _pid->Compute();
  setSpeed((int)Output);
}

int MotorController::getFaultPin() {
  return _fltPin;
}
