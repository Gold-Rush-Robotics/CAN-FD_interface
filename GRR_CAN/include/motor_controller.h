#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <zephyr.h>
#include <device.h>
#include <sys/printk.h>
#include <string>

class GRRMotor {
private:
  int DIR;
  int PWM;
  int SLP;
  int FLT;
  int CS;
  std::string joint_name;
  std::string control_type;

public:
  GRRMotor(int DIR, int PWM, int SLP, int FLT, int EN_OUTA, int EN_OUTB, int CS, const std::string &joint_name, const std::string &control_type);
  GRRMotor();
  void setMotorVelocity(float velocity);
  void setMotorEffort(int effort);
  float getMotorVelocity();
  int getMotorEffort();
  std::string getJointName();
  std::string getControlType();
};

#endif // MOTOR_CONTROLLER_H
