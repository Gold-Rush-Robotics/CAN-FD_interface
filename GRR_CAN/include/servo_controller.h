#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <zephyr.h>
#include <device.h>
#include <sys/printk.h>
#include <string>

class GRRServo {
private:
  int port;
  int upper_limit;
  int lower_limit;
  std::string joint_name;

public:
  GRRServo(int port, int upper_limit, int lower_limit, const std::string &joint_name);
  void setServoPosition(int position);
  std::string getJointName();
};

#endif // SERVO_CONTROLLER_H
