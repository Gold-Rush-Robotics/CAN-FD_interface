#ifndef BLDC_MOTOR_CONTROLLER_H
#define BLDC_MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <Encoder.h>

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
     * Set motor speed using PWM duty cycle
     * @param pwmVal Speed value (-255 to 255), negative = reverse direction
     */
    void setSpeed(int pwmVal);
    
    /**
     * Set motor speed in RPM (mapped to PWM)
     * @param rpm Target RPM (-maxRPM to +maxRPM)
     */
    void setSpeedRPM(float rpm);
    
    /**
     * Set motor speed as percentage
     * @param percent Speed percentage (-100.0 to 100.0)
     */
    void setSpeedPercent(float percent);
    
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
    
    // Encoder
    Encoder* _encoder;
    long _lastEncoderCount;
    unsigned long _lastEncoderTime;
    
    // FG (Frequency Generator) measurement
    volatile unsigned long _fgPulseCount;
    unsigned long _lastFGCount;
    unsigned long _lastFGTime;
    
    // Motor parameters
    int _ticksPerRev;
    float _gearRatio;
    int _fgPulsesPerRev;
    float _maxRPM;
    
    // State
    BrushlessMotorState _state;
    bool _brakeEngaged;
    bool _direction;  // true = forward
    int _currentPWM;
    
    // Fault detection
    unsigned long _lastFaultCheck;
    int _faultBlinkCount;
    
    // Static ISR handling for FG pin
    static BrushlessMotorController* _instances[4];
    static int _instanceCount;
    int _instanceIndex;
    static void fgISR0();
    static void fgISR1();
    static void fgISR2();
    static void fgISR3();
    void handleFGPulse();
};

#endif