#include "../include/servo_controller.h"


LOG_MODULE_REGISTER(servo_controller, LOG_LEVEL_DBG);

ServoController::ServoController(int port, int upper_limit, int lower_limit, const char *joint_name)
  : port(port), upper_limit(upper_limit), lower_limit(lower_limit), joint_name(joint_name)
{
  LOG_DBG("Constructed servo %s on port %d", joint_name, port);
}

void ServoController::setServoPosition(int position)
{
  if (position <= upper_limit && position >= lower_limit) {
    LOG_INF("Setting servo %s to %d", joint_name, position);
    // Real implementation: use pwm_pin_set_usec() or PWM API
  } else {
    LOG_WRN("Requested servo position %d out of bounds (%d..%d)", position, lower_limit, upper_limit);
  }
}

const char *ServoController::getJointName()
{
  return joint_name;
}
