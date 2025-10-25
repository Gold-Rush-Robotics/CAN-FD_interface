#include "../include/motor_controller.h"
#include <drivers/gpio.h>
#include <drivers/pwm.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(motor_controller, LOG_LEVEL_DBG);

GRRMotor::GRRMotor(int DIR, int PWM, int SLP, int FLT, int EN_OUTA, int EN_OUTB, int CS, const std::string &joint_name, const std::string &control_type)
  : DIR(DIR), PWM(PWM), SLP(SLP), FLT(FLT), CS(CS), joint_name(joint_name), control_type(control_type)
{
  LOG_DBG("Constructed motor %s (DIR=%d PWM=%d)", this->joint_name.c_str(), DIR, PWM);
}

GRRMotor::GRRMotor()
  : DIR(0), PWM(0), SLP(0), FLT(0), CS(0), joint_name("NULL"), control_type("NULL")
{
}

void GRRMotor::setMotorVelocity(float velocity)
{
  // In Zephyr you'd use pwm_pin_set_usec or a pwm device; here we log the requested velocity
  LOG_INF("Set motor %s velocity: %f", joint_name.c_str(), velocity);
}

void GRRMotor::setMotorEffort(int effort)
{
  LOG_INF("Set motor %s effort: %d", joint_name.c_str(), effort);
}

float GRRMotor::getMotorVelocity()
{
  // Placeholder: real implementation requires encoder/counter device
  LOG_DBG("getMotorVelocity called for %s", joint_name.c_str());
  return 0.0f;
}

int GRRMotor::getMotorEffort()
{
  LOG_DBG("getMotorEffort called for %s", joint_name.c_str());
  return 0;
}

std::string GRRMotor::getJointName()
{
  return joint_name;
}

std::string GRRMotor::getControlType()
{
  return control_type;
}
