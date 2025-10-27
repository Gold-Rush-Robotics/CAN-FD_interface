#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <Encoder.h>

class MotorController {
public:
  MotorController(int dirPin, int pwmPin, int slpPin, int fltPin, int encA, int encB, int csPin);
  bool begin();
  void setSpeed(int pwmVal);
  void setSpeedRPM(float rpm);
  float getRPM();
  int getFaultPin();

private:
  int _dirPin, _pwmPin, _slpPin, _fltPin, _encA, _encB, _csPin;
  Encoder* _encoder;
  long _lastEncoderCount = 0;
  unsigned long _lastTime = 0;
  const int _ticksPerRev = 100; // Adjust as needed
  const float _gearRatio = 1.0; // Adjust as needed
};

#endif