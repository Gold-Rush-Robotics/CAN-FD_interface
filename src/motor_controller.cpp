#include "../include/motor_controller.h"

LOG_MODULE_REGISTER(motor_controller, LOG_LEVEL_DBG);

MotorController::MotorController(int DIR, int PWM, int SLP, int FLT, int EN_OUTA, int EN_OUTB, int CS, const char *joint_name, const char *control_type)
  : DIR(DIR), PWM(PWM), SLP(SLP), FLT(FLT), CS(CS), EN_OUTA(EN_OUTA), EN_OUTB(EN_OUTB), joint_name(joint_name), control_type(control_type)
{
  LOG_DBG("Constructed motor %s (DIR=%d PWM=%d)", this->joint_name, DIR, PWM);
}

MotorController::MotorController()
  : DIR(0), PWM(0), SLP(0), FLT(0), CS(0), EN_OUTA(0), EN_OUTB(0), joint_name("NULL"), control_type("NULL")
{
}

void MotorController::setMotorVelocity(float velocity)
{
  LOG_INF("Set motor %s velocity: %f", joint_name, velocity);
}

void MotorController::setMotorEffort(int effort)
{
  LOG_INF("Set motor %s effort: %d", joint_name, effort);
}

float MotorController::getMotorVelocity()
{
  // Placeholder: real implementation requires encoder/counter device
  LOG_DBG("getMotorVelocity called for %s", joint_name);
  return 0.0f;
}

int MotorController::getMotorEffort()
{
  LOG_DBG("getMotorEffort called for %s", joint_name);
  return 0;
}

const char *MotorController::getJointName()
{
  return joint_name;
}

const char *MotorController::getControlType()
{
  return control_type;
}
