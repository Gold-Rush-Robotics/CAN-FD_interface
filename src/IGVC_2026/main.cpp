#include <Arduino.h>
#include "can_interface.h"
#include "bldc_motor_controller.h"

// /* Motor 1 Pin Definitions (A89301 Brushless Driver) */
// #define DIR1 1          // Direction input
// #define SPD1 2          // Speed input (PWM)
// #define BR1 33          // Brake input (HIGH = brake engaged)
// #define FLT1 35         // Fault indicator (active LOW, blink patterns)
// #define EN_OUTA1 14     // Encoder channel A
// #define EN_OUTB1 15     // Encoder channel B
// #define EN_OUTC1 16     // Encoder channel C (index/commutation)
// #define FG1 26          // Frequency generator (speed feedback)

/* Motor 2 Pin Definitions (A89301 Brushless Driver) */
#define DIR2 29         // Direction input
#define SPD2 28         // Speed input (PWM)
#define BR2 7           // Brake input (HIGH = brake engaged)
#define FLT2 36         // Fault indicator (active LOW, blink patterns)
#define EN_OUTA2 17     // Encoder channel A
#define EN_OUTB2 18     // Encoder channel B
#define EN_OUTC2 19     // Encoder channel C (index/commutation)
#define FG2 27          // Frequency generator (speed feedback)

#define DIR4 9         // Direction input
#define SPD4 4         // Speed input (PWM)
#define BR4 13           // Brake input (HIGH = brake engaged)
#define FLT4 10         // Fault indicator (active LOW, blink patterns)
#define EN_OUTA4 23     // Encoder channel A
#define EN_OUTB4 24     // Encoder channel B
#define EN_OUTC4 25     // Encoder channel C (index/commutation)
#define FG4 39          // Frequency generator (speed feedback)



#define DISABLE_CAN 1
#define LED_PIN 13

/* Motor configuration */
#define ENCODER_TICKS_PER_REV 60   // Adjust for your encoder
#define GEAR_RATIO 1.0f             // Adjust for your gearbox
#define FG_PULSES_PER_REV 6         // Typical for brushless motors
#define MAX_RPM 3000.0f             // Maximum motor RPM

// Motor controllers (using A89301 brushless driver interface)
// Constructor: (dirPin, spdPin, brakePin, faultPin, encA, encB, encC, fgPin)
// BrushlessMotorController motor1(DIR1, SPD1, BR1, FLT1, EN_OUTA1, EN_OUTB1, EN_OUTC1, FG1);
BrushlessMotorController leftt_motor(DIR2, SPD2, BR2, FLT2, EN_OUTA2, EN_OUTB2, EN_OUTC2, FG2);
// BrushlessMotorController motor3(DIR3, SPD3, BR3, FLT3, EN_OUTA3, EN_OUTB3, EN_OUTC3, FG3);
BrushlessMotorController right_motor(DIR4, SPD4, BR4, FLT4, EN_OUTA4, EN_OUTB4, EN_OUTC4, FG4);
static unsigned long loopCount = 0;
static unsigned long lastFaultCheck = 0;

// CAN interface
CANInterface canInterface;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000);
  Serial.println("===================================");
  Serial.println("Differential Drive Controller");
  Serial.println("===================================");

    /* Initialize Motor 1 */
    if (!right_motor.begin()) {
        Serial.println("ERROR: right_motor initialization failed");
        while (1);
    }
    right_motor.setEncoderParams(ENCODER_TICKS_PER_REV, GEAR_RATIO);
    right_motor.setFGParams(FG_PULSES_PER_REV);
    right_motor.setMaxRPM(MAX_RPM);
    Serial.println("right_motor initialized successfully");

    /* Initialize Motor 2 */
    if (!leftt_motor.begin()) {
        Serial.println("ERROR: leftt_motor initialization failed");
        while (1);
    }
    leftt_motor.setEncoderParams(ENCODER_TICKS_PER_REV, GEAR_RATIO);
    leftt_motor.setFGParams(FG_PULSES_PER_REV);
    leftt_motor.setMaxRPM(MAX_RPM);
    Serial.println("leftt_motor initialized successfully");

#if !DISABLE_CAN
    if (!canInterface.begin("")) {
        Serial.println("ERROR: CAN initialization failed");
        while (1);
    }
    Serial.println("CAN initialized successfully");
#else
    Serial.println("CAN disabled (DISABLE_CAN=1)");
#endif

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  


}

void loop() {
    loopCount++;

    /* Keep PID velocity loop running continuously */
    right_motor.update();
    leftt_motor.update();

    /* LED heartbeat */
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
    delay(50);

    /* Check for motor faults (every 100ms) */
    if (millis() - lastFaultCheck > 100) {
        lastFaultCheck = millis();
        
        if (right_motor.isFaulted()) {
            BrushlessMotorState state = right_motor.getState();
            Serial.print("WARNING: right_motor fault! State=");
            Serial.println(state);
        }
        
        if (leftt_motor.isFaulted()) {
            BrushlessMotorState state = leftt_motor.getState();
            Serial.print("WARNING: leftt_motor fault! State=");
            Serial.println(state);
        }
    }
    // Set test motor speeds
    right_motor.setDirection(true);  // Forward
    leftt_motor.setDirection(false); // Forward
    leftt_motor.setEffort(20);
    right_motor.setEffort(25);
    // right_motor.setRPM(-20.0f);  // 20 RPM target
    // leftt_motor.setRPM(20.0f);   // 20 RPM target
    // if (loopCount % 200 == 0) {  // Every ~2 seconds
        
    //     float pwm_speed1 = (loopCount / 200) * 10.0f;
    //     float pwm_speed2 = (loopCount / 200) % 2 == 0 ? -150.0f : 150.0f;
    //     motor1.setEffort((int)pwm_speed1);
    //     leftt_motor.setEffort((int)abs(pwm_speed2));
    //     Serial.print("Set Motor1 speed to ");
    //     Serial.print(pwm_speed1);
    //     Serial.print(" RPM, leftt_motor speed to ");
    //     Serial.print(pwm_speed2);
    //     Serial.println(" RPM");
    // } 
    CANJointCommand cmd;
#if !DISABLE_CAN
    if (canInterface.readJointCommand(cmd)) {
        Serial.print("Received CAN command, joint=");
        Serial.print(cmd.joint_name);
        Serial.print(", velocity=");
        Serial.println(cmd.velocity);

        // Differential drive: left motor (right_motor) and right motor (leftt_motor)
        if (cmd.joint_name == "L") {
            Serial.println("Setting Left Motor (right_motor) speed");
            right_motor.setRPM(cmd.velocity);
        } else if (cmd.joint_name == "R") {
            Serial.println("Setting Right Motor (leftt_motor) speed");
            leftt_motor.setRPM(cmd.velocity);
        } else {
            Serial.println("Unknown joint name");
        }
    } else {
    static unsigned long lastNoCmd = 0;
    if (millis() - lastNoCmd > 1000) {
      Serial.println("No CAN command received");
      lastNoCmd = millis();
    }
  }
  #endif

  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 1000) {
    lastHeartbeat = millis();
    #if !DISABLE_CAN
    if (canInterface.sendHeartbeat()) {
      Serial.println("Heartbeat sent");
    } else {
      Serial.println("ERROR: Failed to send heartbeat");
    }
    #else
    Serial.println("Heartbeat skipped (CAN disabled)");
    #endif
  }

    /* Send motor feedback every 100ms */
    static unsigned long lastFeedback = 0;
    if (millis() - lastFeedback > 100) {
        lastFeedback = millis();
        
        /* Get RPM from encoder and FG for comparison */
        // float rpm1_enc = motor1.getRPM();
        // float rpm1_fg = motor1.getRPMFromFG();
        float rpm2_enc = leftt_motor.getRPM();
        float rpm2_fg = leftt_motor.getRPMFromFG();
        float rpm4_enc = right_motor.getRPM();
        float rpm4_fg = right_motor.getRPMFromFG();
        
#if !DISABLE_CAN
        // Send feedback for left and right motors
        canInterface.sendJointFeedback("L", rpm4_enc);
        canInterface.sendJointFeedback("R", rpm2_enc);
#endif
        
        /* Debug output every second */
        static unsigned long lastDebug = 0;
        if (millis() - lastDebug > 1000) {
            lastDebug = millis();
            // Serial.print("M1 Enc:");
            // Serial.print(rpm1_enc);
            // Serial.print(" FG:");
            // Serial.print(rpm1_fg);
            Serial.print(" | M2 Enc:");
            Serial.print(rpm2_enc);
            Serial.print(" FG:");
            Serial.print(rpm2_fg);
            Serial.print(" | M4 Enc:");
            Serial.print(rpm4_enc);
            Serial.print(" FG:");
            Serial.print(rpm4_fg);
            Serial.println(rpm2_fg);
        }
    }
    delay(10);
}