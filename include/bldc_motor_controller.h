#ifndef BLDC_MOTOR_CONTROLLER_H
#define BLDC_MOTOR_CONTROLLER_H

#include <Arduino.h>
#include "pid_controller.h"

/**
 * Brushless Motor Controller for Pololu A89301 Driver
 * 
 * Pin descriptions:
 * - DIR:     Direction input - controls motor phase order (LOW/HIGH)
 * - SPD:     Speed input - analog voltage, PWM duty cycle, or pulse frequency
 *            2.5V analog = max speed, 5V tolerant. Also I2C SCL.
 * - BRAKE:   Brake input - HIGH drives all motor outputs low for electrical braking
 * - FAULT:   Fault indicator - open-drain, normally HIGH (pulled to IOREF),
 *            driven LOW in patterns to indicate faults
 * - FG:      Frequency Generator output - motor speed feedback (tachometer)
 *            Also I2C SDA line, pulled up to IOREF
 * - EN_OUTA/B/C: Encoder inputs for position/velocity feedback
 */

/* Fault codes from A89301 (blink patterns) */
enum BrushlessFaultCode {
    FAULT_NONE = 0,
    FAULT_OVERCURRENT = 1,      // 1 blink
    FAULT_OPEN_LOAD = 2,        // 2 blinks
    FAULT_SHORT_CIRCUIT = 3,    // 3 blinks
    FAULT_OVERTEMP = 4,         // 4 blinks
    FAULT_UNDERVOLTAGE = 5,     // 5 blinks
    FAULT_UNKNOWN = 255
};

/* Motor state */
enum BrushlessMotorState {
    BLDC_STATE_IDLE = 0,
    BLDC_STATE_RUNNING = 1,
    BLDC_STATE_BRAKING = 2,
    BLDC_STATE_FAULT = 3,
    BLDC_STATE_ESTOP = 4
};

class BrushlessMotorController {
public:
    /**
     * Constructor for A89301 brushless motor driver
     * @param dirPin    Direction control pin
     * @param spdPin    Speed control pin (PWM output)
     * @param brakePin  Brake control pin
     * @param faultPin  Fault indicator input (active LOW)
     * @param encA      Encoder channel A (EN_OUTA)
     * @param encB      Encoder channel B (EN_OUTB)
     * @param encC      Encoder channel C (EN_OUTC) - for commutation or index
     * @param fgPin     Frequency generator input (speed feedback from motor)
     */
    BrushlessMotorController(int dirPin, int spdPin, int brakePin, int faultPin, 
                    int encA, int encB, int encC, int fgPin);
    
    ~BrushlessMotorController();
    
    /**
     * Initialize motor controller hardware
     * @return true if initialization successful
     */
    bool begin();
    
    /**
     * Set motor linear velocity in meters/second.
     * Sign controls direction (+ forward, - reverse).
     * Internally converted to RPM using wheel radius.
     * @param velocityMps Target wheel linear velocity in m/s
     */
    void setSpeed(float velocityMps);
    
    /**
     * Set motor speed in RPM.
     * - If PID is enabled: sets the closed-loop target RPM.
     * - If PID is disabled: maps RPM to PWM effort open-loop.
     * @param rpm Target RPM (-maxRPM to +maxRPM)
     */
    void setRPM(float rpm);
    
    /**
     * Set PWM effort directly (duty cycle magnitude only).
     * Direction is controlled separately by setDirection().
     * @param effort PWM effort (0 to 255)
     */
    void setEffort(int effort);
    
    /**
     * Get current motor RPM from encoder
     * @return Current RPM
     */
    float getRPM();
    
    /**
     * Get RPM from frequency generator (FG) pin
     * @return RPM calculated from FG pulses
     */
    float getRPMFromFG();
    
    /**
     * Get current encoder position
     * @return Encoder count
     */
    long getEncoderPosition();
    
    /**
     * Reset encoder position to zero
     */
    void resetEncoder();
    
    /**
     * Engage electrical brake (all phases driven low)
     */
    void brake();
    
    /**
     * Release brake and allow motor to coast
     */
    void releaseBrake();
    
    /**
     * Emergency stop - brake and disable
     */
    void emergencyStop();
    
    /**
     * Enable motor after E-stop
     */
    void enable();
    
    /**
     * Check if motor is in fault condition
     * @return true if fault detected
     */
    bool isFaulted();
    
    /**
     * Get the fault pin number (for external monitoring)
     * @return Fault pin number
     */
    int getFaultPin();
    
    /**
     * Read and decode fault pattern from FAULT pin
     * @return BrushlessFaultCode indicating fault type
     */
    BrushlessFaultCode readFaultCode();
    
    /**
     * Get current motor state
     * @return BrushlessMotorState
     */
    BrushlessMotorState getState();
    
    /**
     * Set direction
     * @param forward true for forward, false for reverse
     */
    void setDirection(bool forward);
    
    /**
     * Configure encoder parameters
     * @param ticksPerRev Encoder ticks per motor revolution
     * @param gearRatio Gear ratio (output/input)
     */
    void setEncoderParams(int ticksPerRev, float gearRatio);
    
    /**
     * Configure FG parameters
     * @param pulsesPerRev Number of FG pulses per motor revolution
     */
    void setFGParams(int pulsesPerRev);
    
    /**
     * Set maximum RPM for speed mapping
     * @param maxRPM Maximum motor RPM
     */
    void setMaxRPM(float maxRPM);

    /* ---- Closed-loop (PID) velocity control ---- */

    /**
     * Run one iteration of the PID velocity loop.
     * Call this from your main loop as fast as possible (or at a fixed rate).
     * Uses encoder RPM as feedback by default; set useEncoderFeedback(false)
     * to use the FG pin instead.
     */
    void update();

    /**
     * Enable or disable closed-loop PID control.
    * When disabled, setSpeed and setRPM work open-loop.
     * @param enabled true to enable PID
     */
    void setPIDEnabled(bool enabled);

    /**
     * Check if PID control is currently active
     */
    bool isPIDEnabled() const { return _pidEnabled; }

    /**
     * Configure PID gains
     */
    void setPIDGains(float kp, float ki, float kd);

    /**
     * Choose feedback source for PID loop
     * @param useEncoder  true = encoder (default), false = FG pin
     */
    void useEncoderFeedback(bool useEncoder);

    /**
    * Get the current target RPM set by setRPM()
     */
    float getTargetRPM() const { return _targetRPM; }

    /**
     * Get a reference to the internal PID controller for advanced tuning
     */
    PIDController& pid() { return _pid; }

private:
    // Pin assignments
    int _dirPin;
    int _spdPin;
    int _brakePin;
    int _faultPin;
    int _encA;
    int _encB;
    int _encC;
    int _fgPin;
    
    // Encoder (manual quadrature decode on A/B/C pins via ISR)
    volatile long _encoderCount;
    long _lastEncoderCount;
    unsigned long _lastEncoderTime;
    volatile uint8_t _lastEncState;  // previous AB state for quadrature lookup
    
    // FG (Frequency Generator) measurement
    volatile unsigned long _fgPulseCount;
    unsigned long _lastFGCount;
    unsigned long _lastFGTime;
    
    // Motor parameters
    int _ticksPerRev;
    float _gearRatio;
    int _fgPulsesPerRev;
    float _maxRPM;
    float _wheelRadiusMeters;
    
    // State
    BrushlessMotorState _state;
    bool _brakeEngaged;
    bool _direction;  // true = forward
    int _currentPWM;
    
    // Fault detection
    unsigned long _lastFaultCheck;
    int _faultBlinkCount;

    // PID velocity control
    PIDController _pid;
    float _targetRPM;
    bool _pidEnabled;
    bool _useEncoderFeedback;   // true = encoder, false = FG
    unsigned long _pidSatStartMs;      // timestamp when saturation began
    static constexpr unsigned long PID_SAT_WARN_MS = 2000;  // warn after 2 s saturated
    
    // Static ISR handling for FG and encoder pins
    static BrushlessMotorController* _instances[4];
    static int _instanceCount;
    int _instanceIndex;

    // FG pin ISRs
    static void fgISR0();
    static void fgISR1();
    static void fgISR2();
    static void fgISR3();
    void handleFGPulse();

    // Encoder A/B pin ISRs (quadrature decode)
    static void encISR0();
    static void encISR1();
    static void encISR2();
    static void encISR3();
    void handleEncoderPulse();

    // Encoder C (index) ISRs
    static void idxISR0();
    static void idxISR1();
    static void idxISR2();
    static void idxISR3();
    void handleIndexPulse();
};

#endif