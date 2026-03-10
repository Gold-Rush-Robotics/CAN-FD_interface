#include "bldc_motor_controller.h"

/* Static member initialization */
BrushlessMotorController* BrushlessMotorController::_instances[4] = {nullptr, nullptr, nullptr, nullptr};
int BrushlessMotorController::_instanceCount = 0;

/* Static ISR handlers for FG pin interrupts */
void BrushlessMotorController::fgISR0() {
    if (_instances[0]) _instances[0]->handleFGPulse();
}
void BrushlessMotorController::fgISR1() {
    if (_instances[1]) _instances[1]->handleFGPulse();
}
void BrushlessMotorController::fgISR2() {
    if (_instances[2]) _instances[2]->handleFGPulse();
}
void BrushlessMotorController::fgISR3() {
    if (_instances[3]) _instances[3]->handleFGPulse();
}

BrushlessMotorController::BrushlessMotorController(int dirPin, int spdPin, int brakePin, int faultPin,
                                 int encA, int encB, int encC, int fgPin)
    : _dirPin(dirPin), _spdPin(spdPin), _brakePin(brakePin), _faultPin(faultPin),
      _encA(encA), _encB(encB), _encC(encC), _fgPin(fgPin),
      _encoder(nullptr), _lastEncoderCount(0), _lastEncoderTime(0),
      _fgPulseCount(0), _lastFGCount(0), _lastFGTime(0),
      _ticksPerRev(100), _gearRatio(1.0f), _fgPulsesPerRev(6), _maxRPM(3000.0f),
      _state(BLDC_STATE_IDLE), _brakeEngaged(false), _direction(true), _currentPWM(0),
      _lastFaultCheck(0), _faultBlinkCount(0), _instanceIndex(-1)
{
    /* Create encoder using channels A and B */
    _encoder = new Encoder(_encA, _encB);
    if (!_encoder) {
        Serial.println("ERROR: Failed to allocate Encoder");
    }
    
    /* Register this instance for ISR handling */
    if (_instanceCount < 4) {
        _instanceIndex = _instanceCount;
        _instances[_instanceCount++] = this;
    } else {
        Serial.println("WARNING: Maximum motor instances (4) reached, FG interrupt not available");
    }
}

BrushlessMotorController::~BrushlessMotorController() {
    /* Disable motor before destruction */
    emergencyStop();
    
    /* Clean up encoder */
    if (_encoder) {
        delete _encoder;
        _encoder = nullptr;
    }
    
    /* Remove from instance list */
    if (_instanceIndex >= 0 && _instanceIndex < 4) {
        _instances[_instanceIndex] = nullptr;
    }
}

bool BrushlessMotorController::begin() {
    if (!_encoder) {
        Serial.println("ERROR: Encoder not initialized");
        return false;
    }
    
    /* Configure direction pin - controls motor phase order */
    pinMode(_dirPin, OUTPUT);
    digitalWrite(_dirPin, LOW);  // Default forward direction
    
    /* Configure speed pin - PWM output for speed control
     * A89301 accepts analog voltage, PWM, or frequency
     * Using PWM mode: duty cycle controls speed
     * Set PWM frequency to 32kHz for smooth motor operation
     */
    pinMode(_spdPin, OUTPUT);
    analogWriteFrequency(_spdPin, 32000);  // 32kHz PWM frequency
    analogWrite(_spdPin, 0);  // Start stopped
    
    /* Configure brake pin - HIGH = electrical brake engaged
     * When HIGH, all motor phases driven low
     */
    pinMode(_brakePin, OUTPUT);
    digitalWrite(_brakePin, LOW);  // Start with brake released
    _brakeEngaged = false;
    
    /* Configure fault pin - open-drain output from A89301
     * Normally HIGH (pulled up to IOREF)
     * Driven LOW in patterns to indicate faults
     */
    pinMode(_faultPin, INPUT_PULLUP);
    
    /* Configure encoder channel C (index or commutation) */
    pinMode(_encC, INPUT);
    
    /* Configure FG (Frequency Generator) pin for speed feedback
     * This is a tachometer output from the motor/driver
     */
    pinMode(_fgPin, INPUT_PULLUP);
    
    /* Attach interrupt for FG pulse counting */
    void (*isrFunc)() = nullptr;
    switch (_instanceIndex) {
        case 0: isrFunc = fgISR0; break;
        case 1: isrFunc = fgISR1; break;
        case 2: isrFunc = fgISR2; break;
        case 3: isrFunc = fgISR3; break;
    }
    
    if (isrFunc && digitalPinToInterrupt(_fgPin) != NOT_AN_INTERRUPT) {
        attachInterrupt(digitalPinToInterrupt(_fgPin), isrFunc, RISING);
        Serial.print("FG interrupt attached on pin ");
        Serial.println(_fgPin);
    } else {
        Serial.print("WARNING: FG pin ");
        Serial.print(_fgPin);
        Serial.println(" does not support interrupts, using polling");
    }
    
    /* Initialize timing */
    _lastEncoderTime = millis();
    _lastFGTime = millis();
    _fgPulseCount = 0;
    _lastFGCount = 0;
    
    _state = BLDC_STATE_IDLE;
    
    Serial.print("Motor initialized - DIR:");
    Serial.print(_dirPin);
    Serial.print(" SPD:");
    Serial.print(_spdPin);
    Serial.print(" BRK:");
    Serial.print(_brakePin);
    Serial.print(" FLT:");
    Serial.print(_faultPin);
    Serial.print(" FG:");
    Serial.println(_fgPin);
    
    return true;
}

void BrushlessMotorController::handleFGPulse() {
    _fgPulseCount++;
}

void BrushlessMotorController::setSpeed(int pwmVal) {
    /* Check for fault or E-stop state */
    if (_state == BLDC_STATE_FAULT || _state == BLDC_STATE_ESTOP) {
        Serial.println("WARNING: Cannot set speed while faulted or E-stopped");
        return;
    }
    
    /* Constrain PWM value */
    pwmVal = constrain(pwmVal, -255, 255);
    
    /* Release brake if engaged */
    if (_brakeEngaged && pwmVal != 0) {
        releaseBrake();
    }
    
    /* Set direction based on sign */
    _direction = (pwmVal >= 0);
    digitalWrite(_dirPin, _direction ? LOW : HIGH);
    Serial.print("Set direction to ");
    Serial.println(_direction ? "FORWARD" : "REVERSE");
    
    /* Set speed (absolute value) */
    _currentPWM = abs(pwmVal);
    analogWrite(_spdPin, _currentPWM);
    Serial.print("Set speed PWM to ");
    Serial.println(_currentPWM);
    
    /* Update state */
    if (_currentPWM > 0) {
        _state = BLDC_STATE_RUNNING;
    } else {
        _state = BLDC_STATE_IDLE;
    }
}

void BrushlessMotorController::setSpeedRPM(float rpm) {
    /* Map RPM to PWM value */
    float absRPM = abs(rpm);
    if (absRPM > _maxRPM) absRPM = _maxRPM;
    
    int pwmVal = (int)((absRPM / _maxRPM) * 255.0f);
    if (rpm < 0) pwmVal = -pwmVal;
    
    setSpeed(pwmVal);
}

void BrushlessMotorController::setSpeedPercent(float percent) {
    percent = constrain(percent, -100.0f, 100.0f);
    int pwmVal = (int)((percent / 100.0f) * 255.0f);
    setSpeed(pwmVal);
    
}

float BrushlessMotorController::getRPM() {
    if (!_encoder) {
        Serial.println("ERROR: Encoder not available");
        return 0.0f;
    }
    
    unsigned long now = millis();
    unsigned long dt = now - _lastEncoderTime;
    
    /* Avoid division by zero and update at reasonable intervals */
    if (dt < 10) {
        return 0.0f;  // Too soon, return last known value
    }
    
    long encCount = _encoder->read();
    long delta = encCount - _lastEncoderCount;
    
    /* Calculate RPM: (ticks / ticksPerRev) / (dt_ms / 60000ms) */
    float revolutions = (float)delta / ((float)_ticksPerRev * _gearRatio);
    float minutes = dt / 60000.0f;
    float rpm = (minutes > 0) ? (revolutions / minutes) : 0.0f;
    
    /* Update for next calculation */
    _lastEncoderCount = encCount;
    _lastEncoderTime = now;
    
    return rpm;
}

float BrushlessMotorController::getRPMFromFG() {
    unsigned long now = millis();
    unsigned long dt = now - _lastFGTime;
    
    /* Avoid division by zero and update at reasonable intervals */
    if (dt < 50) {
        return 0.0f;
    }
    
    /* Disable interrupts briefly to read pulse count */
    noInterrupts();
    unsigned long pulses = _fgPulseCount;
    _fgPulseCount = 0;
    interrupts();
    
    unsigned long deltaPulses = pulses;  // Already reset to 0
    
    /* Calculate RPM from FG pulses */
    /* RPM = (pulses / pulsesPerRev) / (dt_ms / 60000ms) */
    float revolutions = (float)deltaPulses / (float)_fgPulsesPerRev;
    float minutes = dt / 60000.0f;
    float rpm = (minutes > 0) ? (revolutions / minutes) : 0.0f;
    
    /* Update timing */
    _lastFGTime = now;
    
    return rpm;
}

long BrushlessMotorController::getEncoderPosition() {
    if (_encoder) {
        return _encoder->read();
    }
    return 0;
}

void BrushlessMotorController::resetEncoder() {
    if (_encoder) {
        _encoder->write(0);
        _lastEncoderCount = 0;
    }
}

void BrushlessMotorController::brake() {
    /* Engage electrical brake - all motor phases driven low */
    digitalWrite(_brakePin, HIGH);
    _brakeEngaged = true;
    
    /* Also stop PWM output */
    analogWrite(_spdPin, 0);
    _currentPWM = 0;
    
    _state = BLDC_STATE_BRAKING;
    Serial.println("Motor brake engaged");
}

void BrushlessMotorController::releaseBrake() {
    digitalWrite(_brakePin, LOW);
    _brakeEngaged = false;
    
    if (_state == BLDC_STATE_BRAKING) {
        _state = BLDC_STATE_IDLE;
    }
    Serial.println("Motor brake released");
}

void BrushlessMotorController::emergencyStop() {
    /* Engage brake */
    brake();
    
    /* Set E-stop state - requires explicit enable() to resume */
    _state = BLDC_STATE_ESTOP;
    Serial.println("EMERGENCY STOP activated");
}

void BrushlessMotorController::enable() {
    if (_state == BLDC_STATE_ESTOP) {
        /* Check for faults before enabling */
        if (isFaulted()) {
            Serial.println("ERROR: Cannot enable - motor faulted");
            _state = BLDC_STATE_FAULT;
            return;
        }
        
        releaseBrake();
        _state = BLDC_STATE_IDLE;
        Serial.println("Motor enabled after E-stop");
    }
}

bool BrushlessMotorController::isFaulted() {
    /* FAULT pin is open-drain, normally HIGH, pulled LOW on fault */
    return (digitalRead(_faultPin) == LOW);
}

int BrushlessMotorController::getFaultPin() {
    return _faultPin;
}

BrushlessFaultCode BrushlessMotorController::readFaultCode() {
    if (!isFaulted()) {
        return FAULT_NONE;
    }
    
    /* 
     * The A89301 indicates faults by blinking patterns on the FAULT pin.
     * This is a simplified implementation - a full implementation would
     * need to time the blink patterns to decode the fault type.
     * 
     * Pattern timing from datasheet:
     * - 1 blink = overcurrent
     * - 2 blinks = open load
     * - 3 blinks = short circuit
     * - 4 blinks = over temperature
     * - 5 blinks = undervoltage
     */
    
    /* For now, just return unknown fault - full pattern detection
     * would require a state machine with timing */
    _state = BLDC_STATE_FAULT;
    return FAULT_UNKNOWN;
}

BrushlessMotorState BrushlessMotorController::getState() {
    /* Update state based on fault pin */
    if (isFaulted() && _state != BLDC_STATE_ESTOP) {
        _state = BLDC_STATE_FAULT;
    }
    return _state;
}

void BrushlessMotorController::setDirection(bool forward) {
    _direction = forward;
    digitalWrite(_dirPin, forward ? LOW : HIGH);
}

void BrushlessMotorController::setEncoderParams(int ticksPerRev, float gearRatio) {
    _ticksPerRev = ticksPerRev;
    _gearRatio = gearRatio;
    Serial.print("Encoder params set: ");
    Serial.print(ticksPerRev);
    Serial.print(" ticks/rev, gear ratio ");
    Serial.println(gearRatio);
}

void BrushlessMotorController::setFGParams(int pulsesPerRev) {
    _fgPulsesPerRev = pulsesPerRev;
    Serial.print("FG params set: ");
    Serial.print(pulsesPerRev);
    Serial.println(" pulses/rev");
}

void BrushlessMotorController::setMaxRPM(float maxRPM) {
    _maxRPM = maxRPM;
    Serial.print("Max RPM set to: ");
    Serial.println(maxRPM);
}