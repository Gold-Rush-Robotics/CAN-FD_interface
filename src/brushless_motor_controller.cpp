#include "../include/motor_controller.h"
#include "../include/grr_can.h"
#include <string.h>
#include <math.h>

LOG_MODULE_REGISTER(motor_controller, LOG_LEVEL_DBG);

/* ================================================================
 *   BrushlessMotorController Implementation
 * ================================================================ */

BrushlessMotorController::BrushlessMotorController()
    : pwm_dev(nullptr), pwm_channel(0), pwm_period_ns(50000), // 20kHz default
      gpio_dev(nullptr), enable_pin(0), direction_pin(0), brake_pin(0), fault_pin(0),
      motor_id(0), joint_name("uninitialized"), control_mode(MOTOR_MODE_DISABLED),
      state(MOTOR_STATE_IDLE), target_velocity(0.0f), current_velocity(0.0f),
      target_position(0), current_position(0), target_torque(0.0f), fault_code(0),
      kp(1.0f), ki(0.01f), kd(0.1f), integral_error(0.0f), last_error(0.0f),
      max_velocity(1000.0f), max_acceleration(500.0f), min_position(-100000), max_position(100000)
{
}

BrushlessMotorController::BrushlessMotorController(uint8_t id, const char *name)
    : pwm_dev(nullptr), pwm_channel(0), pwm_period_ns(50000),
      gpio_dev(nullptr), enable_pin(0), direction_pin(0), brake_pin(0), fault_pin(0),
      motor_id(id), joint_name(name), control_mode(MOTOR_MODE_DISABLED),
      state(MOTOR_STATE_IDLE), target_velocity(0.0f), current_velocity(0.0f),
      target_position(0), current_position(0), target_torque(0.0f), fault_code(0),
      kp(1.0f), ki(0.01f), kd(0.1f), integral_error(0.0f), last_error(0.0f),
      max_velocity(1000.0f), max_acceleration(500.0f), min_position(-100000), max_position(100000)
{
    LOG_INF("Created BrushlessMotorController: id=%d name=%s", id, name);
}

BrushlessMotorController::~BrushlessMotorController()
{
    disable();
}

bool BrushlessMotorController::init(const struct device *pwm, uint32_t channel, uint32_t period_ns,
                                    const struct device *gpio, gpio_pin_t en, gpio_pin_t dir,
                                    gpio_pin_t brk, gpio_pin_t flt)
{
    if (!pwm || !gpio) {
        LOG_ERR("Motor %d: Invalid device pointers", motor_id);
        return false;
    }

    if (!device_is_ready(pwm)) {
        LOG_ERR("Motor %d: PWM device not ready", motor_id);
        return false;
    }

    if (!device_is_ready(gpio)) {
        LOG_ERR("Motor %d: GPIO device not ready", motor_id);
        return false;
    }

    pwm_dev = pwm;
    pwm_channel = channel;
    pwm_period_ns = period_ns;
    gpio_dev = gpio;
    enable_pin = en;
    direction_pin = dir;
    brake_pin = brk;
    fault_pin = flt;

    /* Configure GPIO pins */
    int ret = gpio_pin_configure(gpio_dev, enable_pin, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Motor %d: Failed to configure enable pin", motor_id);
        return false;
    }

    ret = gpio_pin_configure(gpio_dev, direction_pin, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Motor %d: Failed to configure direction pin", motor_id);
        return false;
    }

    ret = gpio_pin_configure(gpio_dev, brake_pin, GPIO_OUTPUT_ACTIVE);  // Brake engaged by default
    if (ret < 0) {
        LOG_ERR("Motor %d: Failed to configure brake pin", motor_id);
        return false;
    }

    ret = gpio_pin_configure(gpio_dev, fault_pin, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Motor %d: Failed to configure fault pin", motor_id);
        return false;
    }

    /* Set PWM to 0% duty cycle initially */
    updatePWM(0.0f);

    state = MOTOR_STATE_IDLE;
    LOG_INF("Motor %d (%s): Initialized successfully", motor_id, joint_name);
    return true;
}

void BrushlessMotorController::updatePWM(float duty_cycle)
{
    if (!pwm_dev) {
        return;
    }

    /* Clamp duty cycle to 0-100% */
    if (duty_cycle < 0.0f) duty_cycle = 0.0f;
    if (duty_cycle > 100.0f) duty_cycle = 100.0f;

    uint32_t pulse_ns = (uint32_t)((duty_cycle / 100.0f) * pwm_period_ns);
    
    int ret = pwm_set(pwm_dev, pwm_channel, pwm_period_ns, pulse_ns, 0);
    if (ret < 0) {
        LOG_ERR("Motor %d: Failed to set PWM (ret=%d)", motor_id, ret);
    }
}

float BrushlessMotorController::computePID(float setpoint, float current)
{
    float error = setpoint - current;
    
    /* Accumulate integral with anti-windup */
    integral_error += error;
    float max_integral = 1000.0f;
    if (integral_error > max_integral) integral_error = max_integral;
    if (integral_error < -max_integral) integral_error = -max_integral;
    
    /* Compute derivative */
    float derivative = error - last_error;
    last_error = error;
    
    /* PID output */
    float output = (kp * error) + (ki * integral_error) + (kd * derivative);
    
    return output;
}

void BrushlessMotorController::checkFaults()
{
    if (!gpio_dev) return;

    int fault_state = gpio_pin_get(gpio_dev, fault_pin);
    if (fault_state > 0) {
        if (state != MOTOR_STATE_FAULT) {
            LOG_ERR("Motor %d: Fault detected!", motor_id);
            state = MOTOR_STATE_FAULT;
            fault_code = 1;  // Generic fault
            disable();
        }
    }
}

void BrushlessMotorController::setControlMode(MotorControlMode mode)
{
    control_mode = mode;
    
    /* Reset PID state on mode change */
    integral_error = 0.0f;
    last_error = 0.0f;
    
    LOG_INF("Motor %d: Control mode set to %d", motor_id, mode);
}

void BrushlessMotorController::setVelocity(float velocity)
{
    /* Clamp to limits */
    if (velocity > max_velocity) velocity = max_velocity;
    if (velocity < -max_velocity) velocity = -max_velocity;
    
    target_velocity = velocity;
    
    if (control_mode != MOTOR_MODE_VELOCITY) {
        setControlMode(MOTOR_MODE_VELOCITY);
    }
    
    LOG_DBG("Motor %d: Target velocity = %.2f", motor_id, (double)velocity);
}

void BrushlessMotorController::setPosition(int32_t position)
{
    /* Clamp to limits */
    if (position > max_position) position = max_position;
    if (position < min_position) position = min_position;
    
    target_position = position;
    
    if (control_mode != MOTOR_MODE_POSITION) {
        setControlMode(MOTOR_MODE_POSITION);
    }
    
    LOG_DBG("Motor %d: Target position = %d", motor_id, position);
}

void BrushlessMotorController::setTorque(float torque)
{
    target_torque = torque;
    
    if (control_mode != MOTOR_MODE_TORQUE) {
        setControlMode(MOTOR_MODE_TORQUE);
    }
    
    LOG_DBG("Motor %d: Target torque = %.2f", motor_id, (double)torque);
}

void BrushlessMotorController::enable()
{
    if (state == MOTOR_STATE_FAULT) {
        LOG_WRN("Motor %d: Cannot enable while faulted", motor_id);
        return;
    }
    
    if (gpio_dev) {
        /* Release brake */
        gpio_pin_set(gpio_dev, brake_pin, 0);
        /* Enable motor driver */
        gpio_pin_set(gpio_dev, enable_pin, 1);
    }
    
    state = MOTOR_STATE_RUNNING;
    LOG_INF("Motor %d: Enabled", motor_id);
}

void BrushlessMotorController::disable()
{
    if (gpio_dev) {
        /* Disable motor driver */
        gpio_pin_set(gpio_dev, enable_pin, 0);
        /* Engage brake */
        gpio_pin_set(gpio_dev, brake_pin, 1);
    }
    
    /* Set PWM to 0 */
    updatePWM(0.0f);
    
    if (state != MOTOR_STATE_FAULT && state != MOTOR_STATE_ESTOP) {
        state = MOTOR_STATE_IDLE;
    }
    
    LOG_INF("Motor %d: Disabled", motor_id);
}

void BrushlessMotorController::emergencyStop()
{
    state = MOTOR_STATE_ESTOP;
    
    /* Immediately stop */
    updatePWM(0.0f);
    
    if (gpio_dev) {
        gpio_pin_set(gpio_dev, enable_pin, 0);
        gpio_pin_set(gpio_dev, brake_pin, 1);
    }
    
    target_velocity = 0.0f;
    target_torque = 0.0f;
    
    LOG_WRN("Motor %d: EMERGENCY STOP!", motor_id);
}

void BrushlessMotorController::resetFault()
{
    if (state == MOTOR_STATE_FAULT || state == MOTOR_STATE_ESTOP) {
        fault_code = 0;
        integral_error = 0.0f;
        last_error = 0.0f;
        state = MOTOR_STATE_IDLE;
        LOG_INF("Motor %d: Fault cleared", motor_id);
    }
}

float BrushlessMotorController::getVelocity() const
{
    return current_velocity;
}

int32_t BrushlessMotorController::getPosition() const
{
    return current_position;
}

float BrushlessMotorController::getTorque() const
{
    return target_torque;
}

MotorState BrushlessMotorController::getState() const
{
    return state;
}

uint8_t BrushlessMotorController::getFaultCode() const
{
    return fault_code;
}

uint8_t BrushlessMotorController::getMotorId() const
{
    return motor_id;
}

const char* BrushlessMotorController::getJointName() const
{
    return joint_name;
}

void BrushlessMotorController::setPIDGains(float p, float i, float d)
{
    kp = p;
    ki = i;
    kd = d;
    LOG_INF("Motor %d: PID gains set to P=%.3f I=%.3f D=%.3f", 
            motor_id, (double)p, (double)i, (double)d);
}

void BrushlessMotorController::setLimits(float max_vel, float max_accel, int32_t min_pos, int32_t max_pos)
{
    max_velocity = max_vel;
    max_acceleration = max_accel;
    min_position = min_pos;
    max_position = max_pos;
    LOG_INF("Motor %d: Limits set", motor_id);
}

bool BrushlessMotorController::processCANCommand(const struct can_frame *frame)
{
    if (!frame || frame->dlc < sizeof(MotorCANCommand)) {
        return false;
    }

    const MotorCANCommand *cmd = (const MotorCANCommand *)frame->data;
    
    /* Check if this command is for us */
    if (cmd->motor_id != motor_id && cmd->motor_id != 0xFF) {  // 0xFF = broadcast
        return false;
    }
    
    LOG_DBG("Motor %d: Processing CAN command mode=%d target=%d flags=0x%02X",
            motor_id, cmd->control_mode, cmd->target_value, cmd->flags);
    
    /* Handle flags */
    if (cmd->flags & MOTOR_FLAG_RESET_FAULT) {
        resetFault();
    }
    
    if (cmd->flags & MOTOR_FLAG_ENABLE) {
        enable();
    } else {
        disable();
    }
    
    /* Handle direction */
    if (gpio_dev && (cmd->flags & MOTOR_FLAG_DIRECTION)) {
        gpio_pin_set(gpio_dev, direction_pin, 1);
    } else if (gpio_dev) {
        gpio_pin_set(gpio_dev, direction_pin, 0);
    }
    
    /* Set control mode and target */
    switch (cmd->control_mode) {
        case MOTOR_MODE_VELOCITY:
            setVelocity((float)cmd->target_value);
            break;
        case MOTOR_MODE_POSITION:
            setPosition((int32_t)cmd->target_value);
            break;
        case MOTOR_MODE_TORQUE:
            setTorque((float)cmd->target_value / 100.0f);  // Scale from fixed point
            break;
        case MOTOR_MODE_DISABLED:
            disable();
            break;
    }
    
    return true;
}

bool BrushlessMotorController::sendCANStatus(const struct device *can_dev)
{
    if (!can_dev) {
        return false;
    }
    
    MotorCANStatus status;
    status.motor_id = motor_id;
    status.state = (uint8_t)state;
    status.current_velocity = (int16_t)current_velocity;
    status.current_position = (int16_t)(current_position & 0xFFFF);
    status.fault_code = fault_code;
    status.temperature = 25;  // Placeholder - would read from sensor
    
    return send_motor_status(motor_id, status.state, status.current_velocity,
                            status.current_position, status.fault_code, status.temperature);
}

void BrushlessMotorController::update(float dt)
{
    /* Check for faults */
    checkFaults();
    
    if (state != MOTOR_STATE_RUNNING) {
        return;
    }
    
    float output = 0.0f;
    
    switch (control_mode) {
        case MOTOR_MODE_VELOCITY:
            output = computePID(target_velocity, current_velocity);
            break;
            
        case MOTOR_MODE_POSITION:
            output = computePID((float)target_position, (float)current_position);
            break;
            
        case MOTOR_MODE_TORQUE:
            /* Direct torque control - map torque to PWM duty cycle */
            output = target_torque * 100.0f;  // Simple scaling
            break;
            
        case MOTOR_MODE_DISABLED:
        default:
            output = 0.0f;
            break;
    }
    
    /* Determine direction */
    bool reverse = (output < 0.0f);
    if (gpio_dev) {
        gpio_pin_set(gpio_dev, direction_pin, reverse ? 1 : 0);
    }
    
    /* Apply output as PWM duty cycle (absolute value) */
    float duty = fabsf(output);
    if (duty > 100.0f) duty = 100.0f;
    
    updatePWM(duty);
    
    /* Simulate velocity update (in real system, read from encoder) */
    current_velocity = target_velocity * 0.9f;  // Simulated response
    current_position += (int32_t)(current_velocity * dt);
}

/* ================================================================
 *   Legacy MotorController Implementation (for backwards compatibility)
 * ================================================================ */

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
    LOG_INF("Set motor %s velocity: %f", joint_name, (double)velocity);
}

void MotorController::setMotorEffort(int effort)
{
    LOG_INF("Set motor %s effort: %d", joint_name, effort);
}

float MotorController::getMotorVelocity()
{
    LOG_DBG("getMotorVelocity called for %s", joint_name);
    return 0.0f;
}

int MotorController::getMotorEffort()
{
    LOG_DBG("getMotorEffort called for %s", joint_name);
    return 0;
}

int MotorController::getRPM()
{
    return 0;
}

int MotorController::getCurrent()
{
    return 0;
}

bool MotorController::isFaulted()
{
    return false;
}

const char *MotorController::getJointName()
{
    return joint_name;
}

const char *MotorController::getControlType()
{
    return control_type;
}
