#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

// #include <zephyr.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <stddef.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>



class MotorController {
private:
  int DIR;
  int PWM;
  int SLP;
  int FLT;
  int CS;
  int EN_OUTA;
  int EN_OUTB;

  // Encoder *encoder;
  const char *joint_name;
  const char *control_type;

public:
  MotorController(int DIR, int PWM, int SLP, int FLT, int EN_OUTA, int EN_OUTB, int CS, const char *joint_name, const char *control_type);
  MotorController();
  void setMotorVelocity(float velocity);
  void setMotorEffort(int effort);
  float getMotorVelocity();
  int getMotorEffort();
  int getRPM();
  int getCurrent();
  bool isFaulted();
  const char *getJointName();
  const char *getControlType();
};

#endif // MOTOR_CONTROLLER_H
