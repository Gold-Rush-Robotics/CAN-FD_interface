#include "../include/servo_controller.h"
#include "../include/grr_can.h"
#include <string.h>
#include <math.h>

LOG_MODULE_REGISTER(servo_controller, LOG_LEVEL_DBG);

/* ================================================================
 *   PWMServoController Implementation
 * ================================================================ */

PWMServoController::PWMServoController()
    : pwm_dev(nullptr), pwm_channel(0), servo_id(0), joint_name("uninitialized"),
      state(SERVO_STATE_DISABLED), min_angle(-900), max_angle(900), angle_offset(0),
      target_angle(0), current_angle(0), move_speed(180), enabled(false),
      torque_enabled(true), fault_code(0), pwm_period_us(SERVO_PWM_PERIOD_US),
      pwm_min_us(SERVO_PWM_MIN_US), pwm_max_us(SERVO_PWM_MAX_US)
{
}

PWMServoController::PWMServoController(uint8_t id, const char *name)
    : pwm_dev(nullptr), pwm_channel(0), servo_id(id), joint_name(name),
      state(SERVO_STATE_DISABLED), min_angle(-900), max_angle(900), angle_offset(0),
      target_angle(0), current_angle(0), move_speed(180), enabled(false),
      torque_enabled(true), fault_code(0), pwm_period_us(SERVO_PWM_PERIOD_US),
      pwm_min_us(SERVO_PWM_MIN_US), pwm_max_us(SERVO_PWM_MAX_US)
{
    LOG_INF("Created PWMServoController: id=%d name=%s", id, name);
}

PWMServoController::~PWMServoController()
{
    disable();
}

bool PWMServoController::init(const struct device *pwm, uint32_t channel)
{
    return init(pwm, channel, SERVO_PWM_MIN_US, SERVO_PWM_MAX_US, SERVO_PWM_PERIOD_US);
}

bool PWMServoController::init(const struct device *pwm, uint32_t channel,
                              uint32_t min_us, uint32_t max_us, uint32_t period_us)
{
    if (!pwm) {
        LOG_ERR("Servo %d: Invalid PWM device pointer", servo_id);
        return false;
    }

    if (!device_is_ready(pwm)) {
        LOG_ERR("Servo %d: PWM device not ready", servo_id);
        return false;
    }

    pwm_dev = pwm;
    pwm_channel = channel;
    pwm_min_us = min_us;
    pwm_max_us = max_us;
    pwm_period_us = period_us;

    /* Move to center position initially */
    target_angle = 0;
    current_angle = 0;
    updatePWM();

    state = SERVO_STATE_IDLE;
    enabled = true;
    
    LOG_INF("Servo %d (%s): Initialized on channel %d", servo_id, joint_name, channel);
    return true;
}

uint32_t PWMServoController::angleToMicroseconds(int16_t angle)
{
    /* angle is in 0.1 degrees, range typically -900 to +900 (-90 to +90 degrees) */
    /* Map angle range to PWM pulse width range */
    
    int16_t clamped_angle = angle;
    if (clamped_angle < min_angle) clamped_angle = min_angle;
    if (clamped_angle > max_angle) clamped_angle = max_angle;
    
    /* Apply offset */
    clamped_angle += angle_offset;
    
    /* Map from angle range to microseconds */
    /* -900 to +900 (0.1 deg) maps to pwm_min_us to pwm_max_us */
    float normalized = (float)(clamped_angle - min_angle) / (float)(max_angle - min_angle);
    uint32_t pulse_us = pwm_min_us + (uint32_t)(normalized * (pwm_max_us - pwm_min_us));
    
    return pulse_us;
}

int16_t PWMServoController::microsecondsToAngle(uint32_t us)
{
    if (us < pwm_min_us) us = pwm_min_us;
    if (us > pwm_max_us) us = pwm_max_us;
    
    float normalized = (float)(us - pwm_min_us) / (float)(pwm_max_us - pwm_min_us);
    int16_t angle = min_angle + (int16_t)(normalized * (max_angle - min_angle));
    
    return angle - angle_offset;
}

void PWMServoController::updatePWM()
{
    if (!pwm_dev || !enabled) {
        return;
    }

    uint32_t pulse_us = angleToMicroseconds(current_angle);
    
    /* Convert microseconds to nanoseconds for Zephyr PWM API */
    uint32_t period_ns = pwm_period_us * 1000;
    uint32_t pulse_ns = pulse_us * 1000;
    
    int ret = pwm_set(pwm_dev, pwm_channel, period_ns, pulse_ns, 0);
    if (ret < 0) {
        LOG_ERR("Servo %d: Failed to set PWM (ret=%d)", servo_id, ret);
        fault_code = 1;
        state = SERVO_STATE_FAULT;
    }
}

void PWMServoController::setAngleLimits(int16_t min_deg_10, int16_t max_deg_10)
{
    min_angle = min_deg_10;
    max_angle = max_deg_10;
    LOG_INF("Servo %d: Angle limits set to [%d, %d] (0.1 deg)", 
            servo_id, min_angle, max_angle);
}

void PWMServoController::setAngleOffset(int16_t offset_deg_10)
{
    angle_offset = offset_deg_10;
    LOG_INF("Servo %d: Angle offset set to %d (0.1 deg)", servo_id, angle_offset);
}

void PWMServoController::setMoveSpeed(uint16_t deg_per_sec)
{
    move_speed = deg_per_sec;
    LOG_DBG("Servo %d: Move speed set to %d deg/s", servo_id, move_speed);
}

void PWMServoController::setPWMTiming(uint32_t min_us, uint32_t max_us)
{
    pwm_min_us = min_us;
    pwm_max_us = max_us;
    LOG_INF("Servo %d: PWM timing set to [%d, %d] us", servo_id, min_us, max_us);
}

void PWMServoController::setAngle(int16_t angle_deg_10)
{
    /* Clamp to limits */
    if (angle_deg_10 < min_angle) angle_deg_10 = min_angle;
    if (angle_deg_10 > max_angle) angle_deg_10 = max_angle;
    
    target_angle = angle_deg_10;
    
    if (state == SERVO_STATE_IDLE || state == SERVO_STATE_HOLDING) {
        state = SERVO_STATE_MOVING;
    }
    
    LOG_DBG("Servo %d: Target angle set to %d (0.1 deg)", servo_id, target_angle);
}

void PWMServoController::setAngleDegrees(float angle_deg)
{
    setAngle((int16_t)(angle_deg * 10.0f));
}

void PWMServoController::setAnglePercent(float percent)
{
    /* 0% = min_angle, 100% = max_angle */
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    
    int16_t angle = min_angle + (int16_t)((percent / 100.0f) * (max_angle - min_angle));
    setAngle(angle);
}

void PWMServoController::center()
{
    setAngle(0);
}

void PWMServoController::enable()
{
    enabled = true;
    if (state == SERVO_STATE_DISABLED) {
        state = SERVO_STATE_IDLE;
    }
    updatePWM();
    LOG_INF("Servo %d: Enabled", servo_id);
}

void PWMServoController::disable()
{
    enabled = false;
    state = SERVO_STATE_DISABLED;
    
    /* Stop PWM output */
    if (pwm_dev) {
        pwm_set(pwm_dev, pwm_channel, pwm_period_us * 1000, 0, 0);
    }
    
    LOG_INF("Servo %d: Disabled", servo_id);
}

int16_t PWMServoController::getCurrentAngle() const
{
    return current_angle;
}

int16_t PWMServoController::getTargetAngle() const
{
    return target_angle;
}

float PWMServoController::getCurrentAngleDegrees() const
{
    return (float)current_angle / 10.0f;
}

ServoState PWMServoController::getState() const
{
    return state;
}

uint8_t PWMServoController::getServoId() const
{
    return servo_id;
}

const char* PWMServoController::getJointName() const
{
    return joint_name;
}

bool PWMServoController::isEnabled() const
{
    return enabled;
}

uint8_t PWMServoController::getFaultCode() const
{
    return fault_code;
}

bool PWMServoController::processCANCommand(const struct can_frame *frame)
{
    if (!frame || frame->dlc < sizeof(ServoCANCommand)) {
        return false;
    }

    const ServoCANCommand *cmd = (const ServoCANCommand *)frame->data;
    
    /* Check if this command is for us */
    if (cmd->servo_id != servo_id && cmd->servo_id != 0xFF) {  // 0xFF = broadcast
        return false;
    }
    
    LOG_DBG("Servo %d: Processing CAN command type=%d target=%d speed=%d flags=0x%02X",
            servo_id, cmd->command_type, cmd->target_angle, cmd->speed, cmd->flags);
    
    /* Handle flags */
    if (cmd->flags & SERVO_FLAG_ENABLE) {
        enable();
    } else {
        disable();
        return true;
    }
    
    torque_enabled = (cmd->flags & SERVO_FLAG_TORQUE_ENABLE) != 0;
    
    /* Handle command type */
    switch (cmd->command_type) {
        case 0:  // Position command
            setAngle(cmd->target_angle);
            break;
        case 1:  // Speed setting
            setMoveSpeed(cmd->speed);
            break;
        case 2:  // Enable/disable (already handled above)
            break;
    }
    
    if (cmd->speed > 0) {
        setMoveSpeed(cmd->speed);
    }
    
    return true;
}

bool PWMServoController::sendCANStatus(const struct device *can_dev)
{
    if (!can_dev) {
        return false;
    }
    
    ServoCANStatus status;
    status.servo_id = servo_id;
    status.state = (uint8_t)state;
    status.current_angle = current_angle;
    status.target_angle = target_angle;
    status.load = 0;  // Placeholder - would read from current sensor
    status.fault_code = fault_code;
    
    return send_servo_status(servo_id, status.state, status.current_angle,
                            status.target_angle, status.load, status.fault_code);
}

void PWMServoController::update(float dt)
{
    if (!enabled || state == SERVO_STATE_FAULT || state == SERVO_STATE_DISABLED) {
        return;
    }
    
    /* Calculate maximum angle change for this time step */
    int16_t max_delta = (int16_t)(move_speed * 10.0f * dt);  // Convert deg/s to 0.1deg per dt
    if (max_delta < 1) max_delta = 1;
    
    /* Move current angle towards target */
    int16_t delta = target_angle - current_angle;
    
    if (delta == 0) {
        if (state == SERVO_STATE_MOVING) {
            state = SERVO_STATE_HOLDING;
        }
        return;
    }
    
    state = SERVO_STATE_MOVING;
    
    /* Limit the step size */
    if (delta > max_delta) {
        delta = max_delta;
    } else if (delta < -max_delta) {
        delta = -max_delta;
    }
    
    current_angle += delta;
    
    /* Update PWM output */
    updatePWM();
    
    /* Check if we've reached target */
    if (current_angle == target_angle) {
        state = SERVO_STATE_HOLDING;
    }
}

/* ================================================================
 *   Legacy ServoController Implementation (for backwards compatibility)
 * ================================================================ */

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
