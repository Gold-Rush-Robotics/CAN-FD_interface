#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <stddef.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/can.h>

/* Brushless Motor Control Modes */
enum MotorControlMode {
    MOTOR_MODE_VELOCITY = 0,
    MOTOR_MODE_POSITION = 1,
    MOTOR_MODE_TORQUE = 2,
    MOTOR_MODE_DISABLED = 3
};

/* Motor State */
enum MotorState {
    MOTOR_STATE_IDLE = 0,
    MOTOR_STATE_RUNNING = 1,
    MOTOR_STATE_FAULT = 2,
    MOTOR_STATE_CALIBRATING = 3,
    MOTOR_STATE_ESTOP = 4
};

/* Motor CAN Command Structure (8 bytes) */
struct MotorCANCommand {
    uint8_t motor_id;           // Motor identifier
    uint8_t control_mode;       // MotorControlMode enum
    int16_t target_value;       // Target velocity/position/torque (scaled)
    uint8_t flags;              // Enable, direction, brake flags
    uint8_t reserved[3];        // Reserved for future use
} __attribute__((packed));

/* Motor CAN Status Structure (8 bytes) */
struct MotorCANStatus {
    uint8_t motor_id;           // Motor identifier
    uint8_t state;              // MotorState enum
    int16_t current_velocity;   // Current velocity (scaled)
    int16_t current_position;   // Current position (scaled)
    uint8_t fault_code;         // Fault code if any
    uint8_t temperature;        // Motor temperature in Celsius
} __attribute__((packed));

/* Motor command flags */
#define MOTOR_FLAG_ENABLE       (1 << 0)
#define MOTOR_FLAG_DIRECTION    (1 << 1)
#define MOTOR_FLAG_BRAKE        (1 << 2)
#define MOTOR_FLAG_RESET_FAULT  (1 << 3)

class BrushlessMotorController {
private:
    /* PWM device and channel for motor control */
    const struct device *pwm_dev;
    uint32_t pwm_channel;
    uint32_t pwm_period_ns;
    
    /* GPIO pins */
    const struct device *gpio_dev;
    gpio_pin_t enable_pin;
    gpio_pin_t direction_pin;
    gpio_pin_t brake_pin;
    gpio_pin_t fault_pin;
    
    /* Motor parameters */
    uint8_t motor_id;
    const char *joint_name;
    MotorControlMode control_mode;
    MotorState state;
    
    /* Current values */
    float target_velocity;
    float current_velocity;
    int32_t target_position;
    int32_t current_position;
    float target_torque;
    uint8_t fault_code;
    
    /* PID parameters */
    float kp, ki, kd;
    float integral_error;
    float last_error;
    
    /* Limits */
    float max_velocity;
    float max_acceleration;
    int32_t min_position;
    int32_t max_position;
    
    /* Internal methods */
    void updatePWM(float duty_cycle);
    float computePID(float setpoint, float current);
    void checkFaults();

public:
    BrushlessMotorController();
    BrushlessMotorController(uint8_t id, const char *name);
    ~BrushlessMotorController();
    
    /* Initialization */
    bool init(const struct device *pwm, uint32_t channel, uint32_t period_ns,
              const struct device *gpio, gpio_pin_t en, gpio_pin_t dir, 
              gpio_pin_t brk, gpio_pin_t flt);
    
    /* Control methods */
    void setControlMode(MotorControlMode mode);
    void setVelocity(float velocity);
    void setPosition(int32_t position);
    void setTorque(float torque);
    void enable();
    void disable();
    void emergencyStop();
    void resetFault();
    
    /* Feedback methods */
    float getVelocity() const;
    int32_t getPosition() const;
    float getTorque() const;
    MotorState getState() const;
    uint8_t getFaultCode() const;
    uint8_t getMotorId() const;
    const char* getJointName() const;
    
    /* PID tuning */
    void setPIDGains(float p, float i, float d);
    void setLimits(float max_vel, float max_accel, int32_t min_pos, int32_t max_pos);
    
    /* CAN interface */
    bool processCANCommand(const struct can_frame *frame);
    bool sendCANStatus(const struct device *can_dev);
    
    /* Periodic update - call from control loop */
    void update(float dt);
};

/* Legacy compatibility class */
class MotorController {
private:
    int DIR;
    int PWM;
    int SLP;
    int FLT;
    int CS;
    int EN_OUTA;
    int EN_OUTB;
    const char *joint_name;
    const char *control_type;

public:
    MotorController(int DIR, int PWM, int SLP, int FLT, int EN_OUTA, int EN_OUTB, int CS, const char *joint_name, const char *control_type);
    MotorController();
    void setMotorVelocity(float velocity);
    void setMotorEffort(int effort);
    float getMotorVelocity();
    int getMotorEffort();
    int getRPM();
    int getCurrent();
    bool isFaulted();
    const char *getJointName();
    const char *getControlType();
};

#endif // MOTOR_CONTROLLER_H
