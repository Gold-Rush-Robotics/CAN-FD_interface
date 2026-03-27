#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

/**
 * PID Controller for closed-loop velocity control
 * 
 * Implements a standard PID controller with:
 * - Anti-windup via integral clamping
 * - Derivative kick prevention (derivative on measurement)
 * - Output saturation limits
 * - Configurable update rate
 */
class PIDController {
public:
    /**
     * Construct a PID controller
     * @param kp  Proportional gain
     * @param ki  Integral gain
     * @param kd  Derivative gain
     * @param outputMin  Minimum output value (e.g. 0 for unidirectional PWM)
     * @param outputMax  Maximum output value (e.g. 255 for 8-bit PWM)
     */
    PIDController(float kp = 1.0f, float ki = 0.0f, float kd = 0.0f,
                  float outputMin = 0.0f, float outputMax = 255.0f);

    /**
     * Compute one PID iteration
     * @param setpoint   Desired value (e.g. target RPM)
     * @param measurement  Current measured value (e.g. actual RPM)
     * @return Computed output (clamped to [outputMin, outputMax])
     */
    float compute(float setpoint, float measurement);

    /**
     * Reset internal state (integral sum, previous error, timestamps)
     * Call this when enabling the controller or changing setpoint drastically.
     */
    void reset();

    /* Gain setters */
    void setGains(float kp, float ki, float kd);
    void setKp(float kp);
    void setKi(float ki);
    void setKd(float kd);

    /* Output limit setters */
    void setOutputLimits(float min, float max);

    /* Integral windup limit – defaults to outputMax */
    void setIntegralLimit(float limit);

    /* Minimum dt between compute() calls (ms). Calls faster than this
     * return the last output without recomputing. Default 10 ms. */
    void setMinDt(unsigned long ms);

    /* Getters for tuning / telemetry */
    float getKp() const { return _kp; }
    float getKi() const { return _ki; }
    float getKd() const { return _kd; }
    float getLastError() const { return _lastError; }
    float getIntegral() const { return _integral; }
    float getLastOutput() const { return _lastOutput; }

    /**
     * Check whether the controller output is saturated (at min or max).
     * Useful for detecting when the motor cannot reach the setpoint.
     * @return true if the last output hit a limit
     */
    bool isSaturated() const { return _saturated; }

private:
    float _kp, _ki, _kd;
    float _outputMin, _outputMax;
    float _integralLimit;

    float _integral;
    float _lastMeasurement;  // for derivative-on-measurement
    float _lastError;
    float _lastOutput;

    unsigned long _lastTime;   // micros() of last compute
    unsigned long _minDtUs;    // minimum interval in microseconds
    bool _firstRun;
    bool _saturated;
};

#endif
