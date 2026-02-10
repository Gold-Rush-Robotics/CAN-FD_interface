#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <stddef.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/can.h>

/* Servo State */
enum ServoState {
    SERVO_STATE_IDLE = 0,
    SERVO_STATE_MOVING = 1,
    SERVO_STATE_HOLDING = 2,
    SERVO_STATE_FAULT = 3,
    SERVO_STATE_DISABLED = 4
};

/* Servo CAN Command Structure (8 bytes) */
struct ServoCANCommand {
    uint8_t servo_id;           // Servo identifier
    uint8_t command_type;       // 0=position, 1=speed, 2=enable/disable
    int16_t target_angle;       // Target angle in 0.1 degrees (-1800 to 1800)
    uint16_t speed;             // Movement speed (degrees per second)
    uint8_t flags;              // Enable, torque enable flags
    uint8_t reserved;
} __attribute__((packed));

/* Servo CAN Status Structure (8 bytes) */
struct ServoCANStatus {
    uint8_t servo_id;           // Servo identifier
    uint8_t state;              // ServoState enum
    int16_t current_angle;      // Current angle in 0.1 degrees
    int16_t target_angle;       // Target angle in 0.1 degrees
    uint8_t load;               // Current load percentage
    uint8_t fault_code;         // Fault code if any
} __attribute__((packed));

/* Servo command flags */
#define SERVO_FLAG_ENABLE        (1 << 0)
#define SERVO_FLAG_TORQUE_ENABLE (1 << 1)
#define SERVO_FLAG_REVERSE       (1 << 2)

/* Standard servo PWM timing constants */
#define SERVO_PWM_PERIOD_US      20000   // 20ms = 50Hz
#define SERVO_PWM_MIN_US         500     // 0.5ms = minimum position
#define SERVO_PWM_MAX_US         2500    // 2.5ms = maximum position
#define SERVO_PWM_CENTER_US      1500    // 1.5ms = center position

class PWMServoController {
private:
    /* PWM device and channel */
    const struct device *pwm_dev;
    uint32_t pwm_channel;
    
    /* Servo parameters */
    uint8_t servo_id;
    const char *joint_name;
    ServoState state;
    
    /* Angle limits (in 0.1 degrees) */
    int16_t min_angle;          // e.g., -900 = -90.0 degrees
    int16_t max_angle;          // e.g., 900 = 90.0 degrees
    int16_t angle_offset;       // Calibration offset
    
    /* Current state */
    int16_t target_angle;
    int16_t current_angle;
    uint16_t move_speed;        // degrees per second
    bool enabled;
    bool torque_enabled;
    uint8_t fault_code;
    
    /* PWM timing (microseconds) */
    uint32_t pwm_period_us;
    uint32_t pwm_min_us;
    uint32_t pwm_max_us;
    
    /* Internal methods */
    uint32_t angleToMicroseconds(int16_t angle);
    int16_t microsecondsToAngle(uint32_t us);
    void updatePWM();

public:
    PWMServoController();
    PWMServoController(uint8_t id, const char *name);
    ~PWMServoController();
    
    /* Initialization */
    bool init(const struct device *pwm, uint32_t channel);
    bool init(const struct device *pwm, uint32_t channel, 
              uint32_t min_us, uint32_t max_us, uint32_t period_us);
    
    /* Configuration */
    void setAngleLimits(int16_t min_deg_10, int16_t max_deg_10);
    void setAngleOffset(int16_t offset_deg_10);
    void setMoveSpeed(uint16_t deg_per_sec);
    void setPWMTiming(uint32_t min_us, uint32_t max_us);
    
    /* Control methods */
    void setAngle(int16_t angle_deg_10);
    void setAngleDegrees(float angle_deg);
    void setAnglePercent(float percent);  // 0.0 to 100.0
    void center();
    void enable();
    void disable();
    
    /* Feedback methods */
    int16_t getCurrentAngle() const;
    int16_t getTargetAngle() const;
    float getCurrentAngleDegrees() const;
    ServoState getState() const;
    uint8_t getServoId() const;
    const char* getJointName() const;
    bool isEnabled() const;
    uint8_t getFaultCode() const;
    
    /* CAN interface */
    bool processCANCommand(const struct can_frame *frame);
    bool sendCANStatus(const struct device *can_dev);
    
    /* Periodic update - call from control loop for smooth motion */
    void update(float dt);
};

/* Legacy compatibility class */
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
