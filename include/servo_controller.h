#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

// #include <zephyr/zephyr.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <stddef.h>
#include <zephyr/logging/log.h>

class ServoController {
private:
  int port;
  int upper_limit;
  int lower_limit;
  const char *joint_name;

public:
  ServoController(int port, int upper_limit, int lower_limit, const char *joint_name);
  void setServoPosition(int position);
  const char *getJointName();
};

#endif // SERVO_CONTROLLER_H
