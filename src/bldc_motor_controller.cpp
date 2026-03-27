#include "bldc_motor_controller.h"

const bool DEBUG = false;

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

/* Static ISR handlers for encoder A/B quadrature interrupts */
void BrushlessMotorController::encISR0() {
    if (_instances[0]) _instances[0]->handleEncoderPulse();
}
void BrushlessMotorController::encISR1() {
    if (_instances[1]) _instances[1]->handleEncoderPulse();
}
void BrushlessMotorController::encISR2() {
    if (_instances[2]) _instances[2]->handleEncoderPulse();
}
void BrushlessMotorController::encISR3() {
    if (_instances[3]) _instances[3]->handleEncoderPulse();
}

/* Static ISR handlers for encoder C (index) interrupts */
void BrushlessMotorController::idxISR0() {
    if (_instances[0]) _instances[0]->handleIndexPulse();
}
void BrushlessMotorController::idxISR1() {
    if (_instances[1]) _instances[1]->handleIndexPulse();
}
void BrushlessMotorController::idxISR2() {
    if (_instances[2]) _instances[2]->handleIndexPulse();
}
void BrushlessMotorController::idxISR3() {
    if (_instances[3]) _instances[3]->handleIndexPulse();
}

BrushlessMotorController::BrushlessMotorController(int dirPin, int spdPin, int brakePin, int faultPin,
                                 int encA, int encB, int encC, int fgPin)
    : _dirPin(dirPin), _spdPin(spdPin), _brakePin(brakePin), _faultPin(faultPin),
      _encA(encA), _encB(encB), _encC(encC), _fgPin(fgPin),
      _encoderCount(0), _lastEncoderCount(0), _lastEncoderTime(0), _lastEncState(0),
      _fgPulseCount(0), _lastFGCount(0), _lastFGTime(0),
    _ticksPerRev(100), _gearRatio(1.0f), _fgPulsesPerRev(6), _maxRPM(3000.0f),
    _wheelRadiusMeters(0.075f),
      _state(BLDC_STATE_IDLE), _brakeEngaged(false), _direction(true), _currentPWM(0),
      _lastFaultCheck(0), _faultBlinkCount(0), _instanceIndex(-1),
      _pid(1.0f, 0.1f, 0.01f, 0.0f, 255.0f),
      _targetRPM(0.0f), _pidEnabled(false), _useEncoderFeedback(true),
      _pidSatStartMs(0)
{
    /* Register this instance for ISR handling */
    if (_instanceCount < 4) {
        _instanceIndex = _instanceCount;
        _instances[_instanceCount++] = this;
    } else {
        Serial.println("WARNING: Maximum motor instances (4) reached, interrupts not available");
    }
}

BrushlessMotorController::~BrushlessMotorController() {
    /* Disable motor before destruction */
    emergencyStop();
    if (DEBUG) {
        Serial.println("BrushlessMotorController instance destroyed");
    }
    /* Detach encoder and FG interrupts */
    if (_instanceIndex >= 0) {
        if (digitalPinToInterrupt(_encA) != NOT_AN_INTERRUPT)
            detachInterrupt(digitalPinToInterrupt(_encA));
        if (digitalPinToInterrupt(_encB) != NOT_AN_INTERRUPT)
            detachInterrupt(digitalPinToInterrupt(_encB));
        if (digitalPinToInterrupt(_encC) != NOT_AN_INTERRUPT)
            detachInterrupt(digitalPinToInterrupt(_encC));
        if (digitalPinToInterrupt(_fgPin) != NOT_AN_INTERRUPT)
            detachInterrupt(digitalPinToInterrupt(_fgPin));
    }
    
    /* Remove from instance list */
    if (_instanceIndex >= 0 && _instanceIndex < 4) {
        _instances[_instanceIndex] = nullptr;
    }
}

bool BrushlessMotorController::begin() {
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
    
    /* Configure encoder A, B, C pins as inputs */
    pinMode(_encA, INPUT);
    pinMode(_encB, INPUT);
    pinMode(_encC, INPUT);
    
    /* Seed the quadrature state so the first ISR fires correctly */
    _lastEncState = (digitalRead(_encA) << 1) | digitalRead(_encB);
    _encoderCount = 0;
    
    /* --- Attach encoder A/B interrupts for quadrature decoding --- */
    void (*encFunc)() = nullptr;
    switch (_instanceIndex) {
        case 0: encFunc = encISR0; break;
        case 1: encFunc = encISR1; break;
        case 2: encFunc = encISR2; break;
        case 3: encFunc = encISR3; break;
    }
    
    if (encFunc) {
        if (digitalPinToInterrupt(_encA) != NOT_AN_INTERRUPT) {
            attachInterrupt(digitalPinToInterrupt(_encA), encFunc, CHANGE);
            Serial.print("Encoder A interrupt attached on pin ");
            Serial.println(_encA);
        } else {
            Serial.print("WARNING: Encoder A pin ");
            Serial.print(_encA);
            Serial.println(" does not support interrupts");
        }
        if (digitalPinToInterrupt(_encB) != NOT_AN_INTERRUPT) {
            attachInterrupt(digitalPinToInterrupt(_encB), encFunc, CHANGE);
            Serial.print("Encoder B interrupt attached on pin ");
            Serial.println(_encB);
        } else {
            Serial.print("WARNING: Encoder B pin ");
            Serial.print(_encB);
            Serial.println(" does not support interrupts");
        }
    }
    
    /* --- Attach encoder C (index) interrupt --- */
    void (*idxFunc)() = nullptr;
    switch (_instanceIndex) {
        case 0: idxFunc = idxISR0; break;
        case 1: idxFunc = idxISR1; break;
        case 2: idxFunc = idxISR2; break;
        case 3: idxFunc = idxISR3; break;
    }
    
    if (idxFunc && digitalPinToInterrupt(_encC) != NOT_AN_INTERRUPT) {
        attachInterrupt(digitalPinToInterrupt(_encC), idxFunc, RISING);
        Serial.print("Encoder C (index) interrupt attached on pin ");
        Serial.println(_encC);
    } else {
        Serial.print("WARNING: Encoder C pin ");
        Serial.print(_encC);
        Serial.println(" does not support interrupts or no ISR slot");
    }
    
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
    Serial.print(" ENC:");
    Serial.print(_encA);
    Serial.print("/");
    Serial.print(_encB);
    Serial.print("/");
    Serial.print(_encC);
    Serial.print(" FG:");
    Serial.println(_fgPin);
    
    return true;
}

void BrushlessMotorController::handleFGPulse() {
    _fgPulseCount++;
}

/**
 * Quadrature decode on A/B pin change.
 * Uses a 4-entry lookup indexed by (prevState << 2 | newState)
 * to determine +1 / -1 / 0 count change.
 *   AB state: 00=0, 01=1, 10=2, 11=3
 */
void BrushlessMotorController::handleEncoderPulse() {
    /*  Quadrature direction lookup table
     *  Index = (oldAB << 2) | newAB   (4-bit, 0-15)
     *  Value: 0 = no change / invalid, +1 = forward, -1 = reverse
     */
    static const int8_t quadLUT[16] = {
    //  new:  00  01  10  11
    /* old 00 */ 0, +1, -1,  0,
    /* old 01 */-1,  0,  0, +1,
    /* old 10 */+1,  0,  0, -1,
    /* old 11 */ 0, -1, +1,  0
    };

    uint8_t newState = (digitalRead(_encA) << 1) | digitalRead(_encB);
    uint8_t idx = (_lastEncState << 2) | newState;
    _encoderCount += quadLUT[idx];
    _lastEncState = newState;
}

/**
 * Encoder C (index) pulse handler.
 * The index pulse fires once per revolution and can be used
 * to correct accumulated quadrature error.
 * For now we simply log it; override for custom behaviour.
 */
void BrushlessMotorController::handleIndexPulse() {
    /* Optional: reset encoder count to a known reference every revolution.
     * Uncomment the line below if your application needs it:
     * _encoderCount = 0;
     */
}

void BrushlessMotorController::setSpeed(float velocityMps) {
    if (_wheelRadiusMeters <= 0.0f) {
        Serial.println("ERROR: Invalid wheel radius, cannot convert m/s to RPM");
        return;
    }

    const float rpm = (velocityMps * 60.0f) / (2.0f * PI * _wheelRadiusMeters);
    setRPM(rpm);
}

void BrushlessMotorController::setEffort(int effort) {
    /* Check for fault or E-stop state */
    if (_state == BLDC_STATE_FAULT || _state == BLDC_STATE_ESTOP) {
        Serial.println("WARNING: Cannot set effort while faulted or E-stopped");
        return;
    }

    /* Constrain PWM value to effort range */
    effort = constrain(effort, 0, 255);
    
    /* Release brake if engaged */
    if (_brakeEngaged && effort != 0) {
        releaseBrake();
    }

    /* Set speed (absolute value) */
    _currentPWM = effort;
    analogWrite(_spdPin, _currentPWM);
    if(DEBUG){
        Serial.print("Set effort PWM to ");
        Serial.println(_currentPWM);
    }

    
    /* Update state */
    if (_currentPWM > 0) {
        _state = BLDC_STATE_RUNNING;
    } else {
        _state = BLDC_STATE_IDLE;
    }
}

void BrushlessMotorController::setRPM(float rpm) {
    _targetRPM = constrain(rpm, -_maxRPM, _maxRPM);
    _pid.reset();          // avoid integral kick on setpoint change
    _pidSatStartMs = 0;    // reset saturation timer

    if (_pidEnabled) {
        return;
    }

    bool forward = (_targetRPM >= 0.0f);
    setDirection(forward);

    float absRPM = fabs(_targetRPM);
    int effort = (int)((absRPM / _maxRPM) * 255.0f);
    setEffort(effort);
}

float BrushlessMotorController::getRPM() {
    unsigned long now = millis();
    unsigned long dt = now - _lastEncoderTime;
    
    /* Avoid division by zero and update at reasonable intervals */
    if (dt < 10) {
        return 0.0f;  // Too soon, return last known value
    }
    
    /* Atomically read the encoder count */
    noInterrupts();
    long encCount = _encoderCount;
    interrupts();
    
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
    noInterrupts();
    long count = _encoderCount;
    interrupts();
    return count;
}

void BrushlessMotorController::resetEncoder() {
    noInterrupts();
    _encoderCount = 0;
    interrupts();
    _lastEncoderCount = 0;
}

void BrushlessMotorController::brake() {
    /* Engage electrical brake - all motor phases driven low */
    digitalWrite(_brakePin, HIGH);
    _brakeEngaged = true;
    
    /* Also stop PWM output */
    analogWrite(_spdPin, 0);
    _currentPWM = 0;
    
    _state = BLDC_STATE_BRAKING;
    if(DEBUG){
        Serial.println("Motor brake engaged");
    }
}

void BrushlessMotorController::releaseBrake() {
    digitalWrite(_brakePin, LOW);
    _brakeEngaged = false;
    
    if (_state == BLDC_STATE_BRAKING) {
        _state = BLDC_STATE_IDLE;
    }
    if(DEBUG){
        Serial.println("Motor brake released");
    }
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
    if(DEBUG){
        Serial.print("Encoder params set: ");
        Serial.print(ticksPerRev);
        Serial.print(" ticks/rev, gear ratio ");
        Serial.println(gearRatio, 2);
    }
}

void BrushlessMotorController::setFGParams(int pulsesPerRev) {
    _fgPulsesPerRev = pulsesPerRev;
    if(DEBUG){
        Serial.print("FG params set: ");
        Serial.print(pulsesPerRev);
        Serial.println(" pulses/rev");
    }
}

void BrushlessMotorController::setMaxRPM(float maxRPM) {
    _maxRPM = maxRPM;
    if(DEBUG){
        Serial.print("Max RPM set to ");
        Serial.println(maxRPM, 1);
    }
}

/* ========== Closed-loop PID velocity control ========== */

void BrushlessMotorController::update() {
    if (!_pidEnabled) return;

    /* Don't run PID when faulted or E-stopped */
    if (_state == BLDC_STATE_FAULT || _state == BLDC_STATE_ESTOP) return;

    /* Get current RPM from the selected feedback source */
    float currentRPM = _useEncoderFeedback ? getRPM() : getRPMFromFG();

    /* PID works on absolute RPM; direction handled separately */
    float absTarget = fabs(_targetRPM);
    float absRPM    = fabs(currentRPM);

    float output = _pid.compute(absTarget, absRPM);

    /* Apply direction */
    bool forward = (_targetRPM >= 0.0f);
    _direction = forward;
    digitalWrite(_dirPin, forward ? LOW : HIGH);

    /* Write PWM */
    int pwmVal = (int)output;
    if (absTarget == 0.0f) pwmVal = 0;  // explicit zero when target is 0
    pwmVal = constrain(pwmVal, 0, 255);

    /* Release brake if needed */
    if (_brakeEngaged && pwmVal != 0) {
        releaseBrake();
    }

    analogWrite(_spdPin, pwmVal);
    _currentPWM = pwmVal;
    _state = (pwmVal > 0) ? BLDC_STATE_RUNNING : BLDC_STATE_IDLE;

    /* --- Max-speed saturation warning --- */
    if (_pid.isSaturated() && absTarget > 0.0f) {
        if (_pidSatStartMs == 0) {
            _pidSatStartMs = millis();
        } else if (millis() - _pidSatStartMs >= PID_SAT_WARN_MS) {
            float error = absTarget - absRPM;
            Serial.print("WARNING: Motor at max output but ");
            Serial.print(error, 1);
            Serial.print(" RPM short of target (");
            Serial.print(absTarget, 1);
            Serial.print(" RPM). Current: ");
            Serial.print(absRPM, 1);
            Serial.println(" RPM");
            /* Only warn once per saturation event */
            _pidSatStartMs = millis();
        }
    } else {
        _pidSatStartMs = 0;  // reset when no longer saturated
    }
}

void BrushlessMotorController::setPIDEnabled(bool enabled) {
    _pidEnabled = enabled;
    if (enabled) {
        _pid.reset();
        _pidSatStartMs = 0;
        if(DEBUG){
            Serial.println("PID velocity control ENABLED");
        }
    } else {
        if(DEBUG){
            Serial.println("PID velocity control DISABLED");
        }
    }
}

void BrushlessMotorController::setPIDGains(float kp, float ki, float kd) {
    _pid.setGains(kp, ki, kd);
    if(DEBUG){
        Serial.print("PID gains set: Kp=");
        Serial.print(kp, 4);
        Serial.print(" Ki=");
        Serial.print(ki, 4);
        Serial.print(" Kd=");
        Serial.println(kd, 4);
    }
}

void BrushlessMotorController::useEncoderFeedback(bool useEncoder) {
    _useEncoderFeedback = useEncoder;
    if(DEBUG){
        Serial.print("PID feedback source: ");
        Serial.println(useEncoder ? "Encoder" : "FG pin");
    }
}