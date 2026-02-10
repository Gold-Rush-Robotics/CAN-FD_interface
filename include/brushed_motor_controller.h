#ifndef BRUSHED_MOTOR_CONTROLLER_H
#define BRUSHED_MOTOR_CONTROLLER_H

#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <stddef.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/can.h>

/* ================================================================
 *   Brushed Motor Control Modes & States
 * ================================================================
 * Brushed motors are simpler than brushless:
 *   - Driven via H-bridge (DIR + PWM + SLP + FLT)
 *   - Current sense via analog CS pin
 *   - Optional quadrature encoder (EN_OUTA / EN_OUTB)
 * ================================================================ */

enum BrushedMotorControlMode {
    BRUSHED_MODE_VELOCITY = 0,    /* Closed-loop velocity (requires encoder) */
    BRUSHED_MODE_POSITION = 1,    /* Closed-loop position (requires encoder) */
    BRUSHED_MODE_EFFORT   = 2,    /* Open-loop PWM duty cycle */
    BRUSHED_MODE_DISABLED = 3
};

enum BrushedMotorState {
    BRUSHED_STATE_IDLE        = 0,
    BRUSHED_STATE_RUNNING     = 1,
    BRUSHED_STATE_FAULT       = 2,
    BRUSHED_STATE_CALIBRATING = 3,
    BRUSHED_STATE_ESTOP       = 4
};

/* Brushed Motor CAN Command (8 bytes) – same wire format as brushless */
struct BrushedMotorCANCommand {
    uint8_t  motor_id;       /* Motor identifier                           */
    uint8_t  control_mode;   /* BrushedMotorControlMode enum               */
    int16_t  target_value;   /* Velocity (RPM) / position (ticks) / effort */
    uint8_t  flags;          /* Enable, direction, brake, reset-fault      */
    uint8_t  reserved[3];
} __attribute__((packed));

/* Brushed Motor CAN Status (8 bytes) */
struct BrushedMotorCANStatus {
    uint8_t  motor_id;
    uint8_t  state;          /* BrushedMotorState enum    */
    int16_t  current_velocity;
    int16_t  current_position;
    uint8_t  fault_code;
    uint8_t  temperature;    /* °C, 0 if not available    */
} __attribute__((packed));

/* Command flag bits (identical to brushless for CAN interoperability) */
#define BRUSHED_FLAG_ENABLE       (1 << 0)
#define BRUSHED_FLAG_DIRECTION    (1 << 1)
#define BRUSHED_FLAG_BRAKE        (1 << 2)
#define BRUSHED_FLAG_RESET_FAULT  (1 << 3)

/* Fault codes */
#define BRUSHED_FAULT_NONE        0x00
#define BRUSHED_FAULT_OVERCURRENT 0x01
#define BRUSHED_FAULT_DRIVER      0x02
#define BRUSHED_FAULT_STALL       0x03
#define BRUSHED_FAULT_ENCODER     0x04

class BrushedMotorController {
private:
    /* ---- H-Bridge Interface ---- */
    const struct device *pwm_dev;
    uint32_t pwm_channel;
    uint32_t pwm_period_ns;        /* PWM period (default 50 µs → 20 kHz) */

    const struct device *gpio_dev;
    gpio_pin_t dir_pin;            /* Direction */
    gpio_pin_t slp_pin;            /* Sleep / enable (active-low on many drivers) */
    gpio_pin_t flt_pin;            /* Fault input (active-low) */

    /* ---- Current Sense ---- */
    const struct device *adc_dev;
    uint8_t  cs_adc_channel;
    int32_t  current_ma;           /* Last sampled motor current in mA */
    int32_t  overcurrent_ma;       /* Trip threshold */

    /* ---- Quadrature Encoder ---- */
    const struct device *enc_gpio_dev;
    gpio_pin_t enc_a_pin;
    gpio_pin_t enc_b_pin;
    volatile int32_t encoder_ticks;
    int32_t  ticks_per_rev;        /* Encoder resolution */

    /* ---- Identity ---- */
    uint8_t  motor_id;
    const char *joint_name;

    /* ---- Control State ---- */
    BrushedMotorControlMode control_mode;
    BrushedMotorState       state;

    float   target_velocity;       /* RPM */
    float   current_velocity;      /* RPM (computed from encoder) */
    int32_t target_position;       /* Encoder ticks */
    int32_t current_position;      /* Encoder ticks */
    int16_t target_effort;         /* Duty-cycle % × 100 (–10 000 … +10 000) */
    uint8_t fault_code;

    /* ---- PID ---- */
    float kp, ki, kd;
    float integral_error;
    float last_error;

    /* ---- Limits ---- */
    float   max_velocity;          /* RPM */
    float   max_acceleration;      /* RPM / s */
    int32_t min_position;
    int32_t max_position;
    float   max_duty;              /* 0-100 % */

    /* ---- Stall detection ---- */
    uint32_t stall_count;
    uint32_t stall_threshold;      /* Consecutive zero-velocity samples */

    /* ---- Internal helpers ---- */
    void    applyPWM(float duty_pct);
    float   computePID(float setpoint, float measured);
    void    checkFaults();
    void    readCurrentSense();
    void    updateEncoder();

public:
    BrushedMotorController();
    BrushedMotorController(uint8_t id, const char *name);
    ~BrushedMotorController();

    /* ---- Hardware init ---- */
    bool init(const struct device *pwm, uint32_t channel, uint32_t period_ns,
              const struct device *gpio, gpio_pin_t dir, gpio_pin_t slp, gpio_pin_t flt);

    /* Optional: attach encoder */
    bool attachEncoder(const struct device *enc_gpio, gpio_pin_t a, gpio_pin_t b,
                       int32_t ticks_per_revolution);

    /* Optional: attach current-sense ADC */
    bool attachCurrentSense(const struct device *adc, uint8_t channel, int32_t overcurrent_limit_ma);

    /* ---- Control ---- */
    void setControlMode(BrushedMotorControlMode mode);
    void setVelocity(float rpm);
    void setPosition(int32_t ticks);
    void setEffort(int16_t effort);       /* –10 000 … +10 000 */
    void enable();
    void disable();
    void emergencyStop();
    void resetFault();

    /* ---- Feedback ---- */
    float   getVelocity()   const;
    int32_t getPosition()   const;
    int16_t getEffort()     const;
    int32_t getCurrentMA()  const;
    BrushedMotorState getState() const;
    uint8_t getFaultCode()  const;
    uint8_t getMotorId()    const;
    const char* getJointName() const;

    /* ---- Tuning ---- */
    void setPIDGains(float p, float i, float d);
    void setLimits(float max_vel, float max_accel, int32_t min_pos, int32_t max_pos);
    void setMaxDuty(float pct);
    void setStallThreshold(uint32_t samples);
    void setTicksPerRev(int32_t tpr);

    /* ---- CAN interface ---- */
    bool processCANCommand(const struct can_frame *frame);
    bool sendCANStatus(const struct device *can_dev);

    /* ---- Periodic update (call from control loop) ---- */
    void update(float dt);
};

#endif /* BRUSHED_MOTOR_CONTROLLER_H */
