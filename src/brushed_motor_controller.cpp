#include "../include/brushed_motor_controller.h"
#include "../include/grr_can.h"
#include <string.h>
#include <math.h>

LOG_MODULE_REGISTER(brushed_motor, LOG_LEVEL_DBG);

/* ================================================================
 *   Constructors / Destructor
 * ================================================================ */

BrushedMotorController::BrushedMotorController()
    : pwm_dev(nullptr), pwm_channel(0), pwm_period_ns(50000),
      gpio_dev(nullptr), dir_pin(0), slp_pin(0), flt_pin(0),
      adc_dev(nullptr), cs_adc_channel(0), current_ma(0), overcurrent_ma(10000),
      enc_gpio_dev(nullptr), enc_a_pin(0), enc_b_pin(0),
      encoder_ticks(0), ticks_per_rev(1440),
      motor_id(0), joint_name("uninitialized"),
      control_mode(BRUSHED_MODE_DISABLED), state(BRUSHED_STATE_IDLE),
      target_velocity(0.0f), current_velocity(0.0f),
      target_position(0), current_position(0),
      target_effort(0), fault_code(BRUSHED_FAULT_NONE),
      kp(1.0f), ki(0.01f), kd(0.1f),
      integral_error(0.0f), last_error(0.0f),
      max_velocity(5000.0f), max_acceleration(2000.0f),
      min_position(-1000000), max_position(1000000),
      max_duty(100.0f),
      stall_count(0), stall_threshold(50)
{
}

BrushedMotorController::BrushedMotorController(uint8_t id, const char *name)
    : pwm_dev(nullptr), pwm_channel(0), pwm_period_ns(50000),
      gpio_dev(nullptr), dir_pin(0), slp_pin(0), flt_pin(0),
      adc_dev(nullptr), cs_adc_channel(0), current_ma(0), overcurrent_ma(10000),
      enc_gpio_dev(nullptr), enc_a_pin(0), enc_b_pin(0),
      encoder_ticks(0), ticks_per_rev(1440),
      motor_id(id), joint_name(name),
      control_mode(BRUSHED_MODE_DISABLED), state(BRUSHED_STATE_IDLE),
      target_velocity(0.0f), current_velocity(0.0f),
      target_position(0), current_position(0),
      target_effort(0), fault_code(BRUSHED_FAULT_NONE),
      kp(1.0f), ki(0.01f), kd(0.1f),
      integral_error(0.0f), last_error(0.0f),
      max_velocity(5000.0f), max_acceleration(2000.0f),
      min_position(-1000000), max_position(1000000),
      max_duty(100.0f),
      stall_count(0), stall_threshold(50)
{
    LOG_INF("Created BrushedMotorController: id=%d name=%s", id, name);
}

BrushedMotorController::~BrushedMotorController()
{
    disable();
}

/* ================================================================
 *   Hardware Initialisation
 * ================================================================ */

bool BrushedMotorController::init(const struct device *pwm, uint32_t channel,
                                  uint32_t period_ns,
                                  const struct device *gpio,
                                  gpio_pin_t dir, gpio_pin_t slp, gpio_pin_t flt)
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

    pwm_dev       = pwm;
    pwm_channel   = channel;
    pwm_period_ns = period_ns;
    gpio_dev      = gpio;
    dir_pin       = dir;
    slp_pin       = slp;
    flt_pin       = flt;

    /* Direction output */
    int ret = gpio_pin_configure(gpio_dev, dir_pin, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Motor %d: DIR pin config failed (%d)", motor_id, ret);
        return false;
    }

    /* Sleep / enable – start asleep (motor off) */
    ret = gpio_pin_configure(gpio_dev, slp_pin, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Motor %d: SLP pin config failed (%d)", motor_id, ret);
        return false;
    }

    /* Fault input (active-low on most H-bridge drivers) */
    ret = gpio_pin_configure(gpio_dev, flt_pin, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        LOG_ERR("Motor %d: FLT pin config failed (%d)", motor_id, ret);
        return false;
    }

    /* Start with 0 % duty */
    applyPWM(0.0f);

    state = BRUSHED_STATE_IDLE;
    LOG_INF("Motor %d (%s): H-bridge initialised (PWM ch%d, period %u ns)",
            motor_id, joint_name, pwm_channel, pwm_period_ns);
    return true;
}

bool BrushedMotorController::attachEncoder(const struct device *enc_gpio,
                                           gpio_pin_t a, gpio_pin_t b,
                                           int32_t tpr)
{
    if (!enc_gpio || !device_is_ready(enc_gpio)) {
        LOG_ERR("Motor %d: Encoder GPIO device not ready", motor_id);
        return false;
    }

    enc_gpio_dev  = enc_gpio;
    enc_a_pin     = a;
    enc_b_pin     = b;
    ticks_per_rev = tpr;
    encoder_ticks = 0;

    /* Configure encoder pins as inputs */
    int ret = gpio_pin_configure(enc_gpio_dev, enc_a_pin, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Motor %d: Encoder A pin config failed (%d)", motor_id, ret);
        return false;
    }
    ret = gpio_pin_configure(enc_gpio_dev, enc_b_pin, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Motor %d: Encoder B pin config failed (%d)", motor_id, ret);
        return false;
    }

    /*
     * NOTE: For production, set up a GPIO interrupt callback on enc_a_pin
     * to increment / decrement encoder_ticks based on enc_b_pin level.
     * Left as polling here for portability.
     */

    LOG_INF("Motor %d: Encoder attached (A=%d B=%d, %d ticks/rev)",
            motor_id, enc_a_pin, enc_b_pin, ticks_per_rev);
    return true;
}

bool BrushedMotorController::attachCurrentSense(const struct device *adc,
                                                uint8_t channel,
                                                int32_t overcurrent_limit_ma)
{
    if (!adc || !device_is_ready(adc)) {
        LOG_ERR("Motor %d: ADC device not ready", motor_id);
        return false;
    }
    adc_dev        = adc;
    cs_adc_channel = channel;
    overcurrent_ma = overcurrent_limit_ma;

    LOG_INF("Motor %d: Current-sense ADC attached (ch%d, OC limit %d mA)",
            motor_id, channel, overcurrent_limit_ma);
    return true;
}

/* ================================================================
 *   Internal Helpers
 * ================================================================ */

void BrushedMotorController::applyPWM(float duty_pct)
{
    if (!pwm_dev) return;

    if (duty_pct < 0.0f) duty_pct = 0.0f;
    if (duty_pct > max_duty) duty_pct = max_duty;

    uint32_t pulse_ns = (uint32_t)((duty_pct / 100.0f) * pwm_period_ns);

    int ret = pwm_set(pwm_dev, pwm_channel, pwm_period_ns, pulse_ns, 0);
    if (ret < 0) {
        LOG_ERR("Motor %d: PWM set failed (ret=%d)", motor_id, ret);
    }
}

float BrushedMotorController::computePID(float setpoint, float measured)
{
    float error = setpoint - measured;

    /* Anti-windup integral */
    integral_error += error;
    const float max_integral = 1000.0f;
    if (integral_error >  max_integral) integral_error =  max_integral;
    if (integral_error < -max_integral) integral_error = -max_integral;

    float derivative = error - last_error;
    last_error = error;

    return (kp * error) + (ki * integral_error) + (kd * derivative);
}

void BrushedMotorController::readCurrentSense()
{
    if (!adc_dev) return;

    /*
     * Placeholder: A real implementation would call adc_read() with the
     * proper adc_sequence / adc_channel_cfg and convert the raw value
     * to milliamps based on the current-sense resistor value.
     */
    /* current_ma = ...; */
}

void BrushedMotorController::updateEncoder()
{
    if (!enc_gpio_dev) return;

    /*
     * Placeholder: In production this would be replaced by an ISR-driven
     * counter or hardware quadrature decoder.  The ISR increments
     * encoder_ticks on each edge of channel A, using channel B for
     * direction.
     */
    current_position = encoder_ticks;
}

void BrushedMotorController::checkFaults()
{
    /* Hardware fault pin (active-low on most H-bridge ICs) */
    if (gpio_dev) {
        int flt = gpio_pin_get(gpio_dev, flt_pin);
        if (flt == 0) {   /* active-low */
            if (state != BRUSHED_STATE_FAULT) {
                LOG_ERR("Motor %d: Driver fault!", motor_id);
                fault_code = BRUSHED_FAULT_DRIVER;
                state = BRUSHED_STATE_FAULT;
                disable();
            }
            return;
        }
    }

    /* Over-current protection */
    if (adc_dev && current_ma > overcurrent_ma) {
        LOG_ERR("Motor %d: Over-current (%d mA > %d mA)", motor_id,
                (int)current_ma, (int)overcurrent_ma);
        fault_code = BRUSHED_FAULT_OVERCURRENT;
        state = BRUSHED_STATE_FAULT;
        disable();
        return;
    }

    /* Stall detection (motor commanded but encoder not moving) */
    if (enc_gpio_dev && state == BRUSHED_STATE_RUNNING) {
        if (fabsf(current_velocity) < 1.0f && fabsf(target_velocity) > 10.0f) {
            stall_count++;
            if (stall_count >= stall_threshold) {
                LOG_ERR("Motor %d: Stall detected!", motor_id);
                fault_code = BRUSHED_FAULT_STALL;
                state = BRUSHED_STATE_FAULT;
                disable();
            }
        } else {
            stall_count = 0;
        }
    }
}

/* ================================================================
 *   Control Methods
 * ================================================================ */

void BrushedMotorController::setControlMode(BrushedMotorControlMode mode)
{
    control_mode = mode;
    integral_error = 0.0f;
    last_error = 0.0f;
    LOG_INF("Motor %d: Control mode → %d", motor_id, mode);
}

void BrushedMotorController::setVelocity(float rpm)
{
    if (rpm >  max_velocity) rpm =  max_velocity;
    if (rpm < -max_velocity) rpm = -max_velocity;
    target_velocity = rpm;

    if (control_mode != BRUSHED_MODE_VELOCITY) {
        setControlMode(BRUSHED_MODE_VELOCITY);
    }
    LOG_DBG("Motor %d: Target velocity = %.1f RPM", motor_id, (double)rpm);
}

void BrushedMotorController::setPosition(int32_t ticks)
{
    if (ticks > max_position) ticks = max_position;
    if (ticks < min_position) ticks = min_position;
    target_position = ticks;

    if (control_mode != BRUSHED_MODE_POSITION) {
        setControlMode(BRUSHED_MODE_POSITION);
    }
    LOG_DBG("Motor %d: Target position = %d ticks", motor_id, ticks);
}

void BrushedMotorController::setEffort(int16_t effort)
{
    if (effort >  10000) effort =  10000;
    if (effort < -10000) effort = -10000;
    target_effort = effort;

    if (control_mode != BRUSHED_MODE_EFFORT) {
        setControlMode(BRUSHED_MODE_EFFORT);
    }
    LOG_DBG("Motor %d: Target effort = %d", motor_id, effort);
}

void BrushedMotorController::enable()
{
    if (state == BRUSHED_STATE_FAULT) {
        LOG_WRN("Motor %d: Cannot enable while faulted – call resetFault() first", motor_id);
        return;
    }

    if (gpio_dev) {
        gpio_pin_set(gpio_dev, slp_pin, 1);   /* Wake driver */
    }

    state = BRUSHED_STATE_RUNNING;
    LOG_INF("Motor %d: Enabled", motor_id);
}

void BrushedMotorController::disable()
{
    applyPWM(0.0f);

    if (gpio_dev) {
        gpio_pin_set(gpio_dev, slp_pin, 0);   /* Sleep driver */
    }

    if (state != BRUSHED_STATE_FAULT && state != BRUSHED_STATE_ESTOP) {
        state = BRUSHED_STATE_IDLE;
    }

    LOG_INF("Motor %d: Disabled", motor_id);
}

void BrushedMotorController::emergencyStop()
{
    state = BRUSHED_STATE_ESTOP;
    applyPWM(0.0f);

    if (gpio_dev) {
        gpio_pin_set(gpio_dev, slp_pin, 0);
    }

    target_velocity = 0.0f;
    target_effort   = 0;
    LOG_WRN("Motor %d: EMERGENCY STOP!", motor_id);
}

void BrushedMotorController::resetFault()
{
    if (state == BRUSHED_STATE_FAULT || state == BRUSHED_STATE_ESTOP) {
        fault_code = BRUSHED_FAULT_NONE;
        integral_error = 0.0f;
        last_error = 0.0f;
        stall_count = 0;
        state = BRUSHED_STATE_IDLE;
        LOG_INF("Motor %d: Fault cleared", motor_id);
    }
}

/* ================================================================
 *   Feedback Getters
 * ================================================================ */

float   BrushedMotorController::getVelocity()   const { return current_velocity; }
int32_t BrushedMotorController::getPosition()   const { return current_position; }
int16_t BrushedMotorController::getEffort()     const { return target_effort;    }
int32_t BrushedMotorController::getCurrentMA()  const { return current_ma;       }
BrushedMotorState BrushedMotorController::getState()     const { return state;        }
uint8_t BrushedMotorController::getFaultCode()  const { return fault_code;       }
uint8_t BrushedMotorController::getMotorId()    const { return motor_id;         }
const char* BrushedMotorController::getJointName() const { return joint_name;    }

/* ================================================================
 *   Tuning
 * ================================================================ */

void BrushedMotorController::setPIDGains(float p, float i, float d)
{
    kp = p; ki = i; kd = d;
    LOG_INF("Motor %d: PID → P=%.3f I=%.3f D=%.3f",
            motor_id, (double)p, (double)i, (double)d);
}

void BrushedMotorController::setLimits(float max_vel, float max_accel,
                                       int32_t min_pos, int32_t max_pos)
{
    max_velocity     = max_vel;
    max_acceleration = max_accel;
    min_position     = min_pos;
    max_position     = max_pos;
    LOG_INF("Motor %d: Limits updated", motor_id);
}

void BrushedMotorController::setMaxDuty(float pct)
{
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    max_duty = pct;
}

void BrushedMotorController::setStallThreshold(uint32_t samples)
{
    stall_threshold = samples;
}

void BrushedMotorController::setTicksPerRev(int32_t tpr)
{
    ticks_per_rev = (tpr > 0) ? tpr : 1;
}

/* ================================================================
 *   CAN Interface
 * ================================================================ */

bool BrushedMotorController::processCANCommand(const struct can_frame *frame)
{
    if (!frame || frame->dlc < sizeof(BrushedMotorCANCommand)) {
        return false;
    }

    const BrushedMotorCANCommand *cmd =
        reinterpret_cast<const BrushedMotorCANCommand *>(frame->data);

    /* Not for us? */
    if (cmd->motor_id != motor_id && cmd->motor_id != 0xFF) {
        return false;
    }

    LOG_DBG("Motor %d: CAN cmd mode=%d target=%d flags=0x%02X",
            motor_id, cmd->control_mode, cmd->target_value, cmd->flags);

    /* Flags */
    if (cmd->flags & BRUSHED_FLAG_RESET_FAULT) {
        resetFault();
    }
    if (cmd->flags & BRUSHED_FLAG_ENABLE) {
        enable();
    } else {
        disable();
        return true;
    }

    /* Direction override (only meaningful in EFFORT mode) */
    /* Direction is otherwise set automatically by sign of target value */

    switch (cmd->control_mode) {
        case BRUSHED_MODE_VELOCITY:
            setVelocity(static_cast<float>(cmd->target_value));
            break;
        case BRUSHED_MODE_POSITION:
            setPosition(static_cast<int32_t>(cmd->target_value));
            break;
        case BRUSHED_MODE_EFFORT:
            setEffort(cmd->target_value);
            break;
        case BRUSHED_MODE_DISABLED:
            disable();
            break;
    }

    return true;
}

bool BrushedMotorController::sendCANStatus(const struct device *can_dev)
{
    if (!can_dev) return false;

    BrushedMotorCANStatus sts;
    sts.motor_id          = motor_id;
    sts.state             = static_cast<uint8_t>(state);
    sts.current_velocity  = static_cast<int16_t>(current_velocity);
    sts.current_position  = static_cast<int16_t>(current_position & 0xFFFF);
    sts.fault_code        = fault_code;
    sts.temperature       = 0;   /* Placeholder – read from thermistor ADC if available */

    return send_motor_status(motor_id, sts.state, sts.current_velocity,
                             sts.current_position, sts.fault_code, sts.temperature);
}

/* ================================================================
 *   Periodic Update  (call at CONTROL_LOOP_HZ)
 * ================================================================ */

void BrushedMotorController::update(float dt)
{
    /* ---- Sensor reads ---- */
    readCurrentSense();
    updateEncoder();
    checkFaults();

    if (state != BRUSHED_STATE_RUNNING) {
        return;
    }

    float output = 0.0f;

    switch (control_mode) {
        case BRUSHED_MODE_VELOCITY:
            output = computePID(target_velocity, current_velocity);
            break;

        case BRUSHED_MODE_POSITION:
            output = computePID(static_cast<float>(target_position),
                                static_cast<float>(current_position));
            break;

        case BRUSHED_MODE_EFFORT:
            /* Open-loop: target_effort is in [-10000, +10000] → [-100%, +100%] */
            output = static_cast<float>(target_effort) / 100.0f;
            break;

        case BRUSHED_MODE_DISABLED:
        default:
            output = 0.0f;
            break;
    }

    /* Direction */
    bool reverse = (output < 0.0f);
    if (gpio_dev) {
        gpio_pin_set(gpio_dev, dir_pin, reverse ? 1 : 0);
    }

    /* Duty cycle (absolute) */
    float duty = fabsf(output);
    if (duty > max_duty) duty = max_duty;
    applyPWM(duty);

    /* ---- Velocity estimation (encoder) ---- */
    if (enc_gpio_dev && ticks_per_rev > 0) {
        static int32_t prev_ticks = 0;
        int32_t delta = encoder_ticks - prev_ticks;
        prev_ticks = encoder_ticks;
        /* RPM = (delta_ticks / ticks_per_rev) / dt * 60 */
        current_velocity = (static_cast<float>(delta) / ticks_per_rev) / dt * 60.0f;
    } else {
        /* No encoder – simulate a rough first-order response */
        current_velocity += (target_velocity - current_velocity) * 0.1f;
    }

    current_position = encoder_ticks;
}
