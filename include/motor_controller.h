#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <PID_v1.h>
#include <Arduino.h>
#include <Encoder.h>

class MotorController {
public:
  MotorController(int dirPin, int pwmPin, int slpPin, int fltPin, int encA, int encB, int csPin, int starting_direction = 1, double Kp, double Ki, double Kd);
  bool begin();
  void setSpeed(int pwmVal);
  void setSpeedRPM(float rpm);
  void PidSetSpeedRPM(float rpm);
  float getRPM();
  void PidLoop();
  int getFaultPin();

private:
  int _dirPin, _pwmPin, _slpPin, _fltPin, _encA, _encB, _csPin;
  double Input, Output, Setpoint;
  Encoder* _encoder;
  PID* _pid;
  long _lastEncoderCount = 0;
  unsigned long _lastTime = 0;
  const int _ticksPerRev = 100; // Adjust as needed
  const float _gearRatio = 1.0; // Adjust as needed
  int starting_direction = 1; // 1 for forward, -1 for reverse
};

#endif