#include "pid_controller.h"

PIDController::PIDController(float kp, float ki, float kd,
                             float outputMin, float outputMax)
    : _kp(kp), _ki(ki), _kd(kd),
      _outputMin(outputMin), _outputMax(outputMax),
      _integralLimit(outputMax),
      _integral(0.0f), _lastMeasurement(0.0f), _lastError(0.0f), _lastOutput(0.0f),
      _lastTime(0), _minDtUs(10000),  // 10 ms default
      _firstRun(true), _saturated(false)
{
}

float PIDController::compute(float setpoint, float measurement) {
    unsigned long now = micros();

    /* On first call just seed the state and return 0 */
    if (_firstRun) {
        _lastTime = now;
        _lastMeasurement = measurement;
        _lastError = setpoint - measurement;
        _firstRun = false;
        return _lastOutput;
    }

    /* Enforce minimum update interval */
    unsigned long elapsed = now - _lastTime;
    if (elapsed < _minDtUs) {
        return _lastOutput;
    }

    float dt = elapsed / 1e6f;  // seconds
    float error = setpoint - measurement;

    /* --- Proportional term --- */
    float pTerm = _kp * error;

    /* --- Integral term with anti-windup --- */
    _integral += error * dt;

    /* Clamp integral to prevent windup */
    if (_integral > _integralLimit)  _integral = _integralLimit;
    if (_integral < -_integralLimit) _integral = -_integralLimit;

    float iTerm = _ki * _integral;

    /* --- Derivative term (on measurement to avoid derivative kick) --- */
    float dMeasurement = (measurement - _lastMeasurement) / dt;
    float dTerm = -_kd * dMeasurement;  // negative because d(error)/dt = -d(measurement)/dt when setpoint is constant

    /* --- Sum and clamp output --- */
    float output = pTerm + iTerm + dTerm;

    _saturated = false;
    if (output > _outputMax) {
        output = _outputMax;
        _saturated = true;
        /* Back-calculate integral to prevent further windup while saturated */
        _integral -= error * dt;
    } else if (output < _outputMin) {
        output = _outputMin;
        _saturated = true;
        _integral -= error * dt;
    }

    /* --- Store state --- */
    _lastError = error;
    _lastMeasurement = measurement;
    _lastOutput = output;
    _lastTime = now;

    return output;
}

void PIDController::reset() {
    _integral = 0.0f;
    _lastMeasurement = 0.0f;
    _lastError = 0.0f;
    _lastOutput = 0.0f;
    _lastTime = 0;
    _firstRun = true;
    _saturated = false;
}

void PIDController::setGains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PIDController::setKp(float kp) { _kp = kp; }
void PIDController::setKi(float ki) { _ki = ki; }
void PIDController::setKd(float kd) { _kd = kd; }

void PIDController::setOutputLimits(float min, float max) {
    _outputMin = min;
    _outputMax = max;
}

void PIDController::setIntegralLimit(float limit) {
    _integralLimit = limit;
}

void PIDController::setMinDt(unsigned long ms) {
    _minDtUs = ms * 1000UL;
}
