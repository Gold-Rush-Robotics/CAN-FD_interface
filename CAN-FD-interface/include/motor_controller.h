#pragma once
#include <Arduino.h>
#include <Encoder.h>

class MotorController {
public:
  MotorController(int dirPin, int pwmPin, int slpPin, int fltPin,
                  int encA, int encB, int csPin);

  void begin();
  void setSpeed(int pwmVal);
  void setSpeedRPM(float rpm);
  float getRPM();

private:
  int _dirPin, _pwmPin, _slpPin, _fltPin, _encA, _encB, _csPin;
  Encoder* _encoder;
  const int _ticksPerRev = 64;
  const float _gearRatio = 30.0;
  long _lastEncoderCount = 0;
  unsigned long _lastTime = 0;
};
