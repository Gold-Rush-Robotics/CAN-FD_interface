#include <Arduino.h>
#include "can_interface.h"
#include "motor_controller.h"

/* Motor 1 Pin Definitions (A89301 Brushless Driver) */
#define DIR1 1          // Direction input
#define SPD1 2          // Speed input (PWM)
#define BR1 33          // Brake input (HIGH = brake engaged)
#define FLT1 35         // Fault indicator (active LOW, blink patterns)
#define EN_OUTA1 14     // Encoder channel A
#define EN_OUTB1 15     // Encoder channel B
#define EN_OUTC1 16     // Encoder channel C (index/commutation)
#define FG1 26          // Frequency generator (speed feedback)

/* Motor 2 Pin Definitions (A89301 Brushless Driver) */
#define DIR2 29         // Direction input
#define SPD2 28         // Speed input (PWM)
#define BR2 7           // Brake input (HIGH = brake engaged)
#define FLT2 36         // Fault indicator (active LOW, blink patterns)
#define EN_OUTA2 17     // Encoder channel A
#define EN_OUTB2 18     // Encoder channel B
#define EN_OUTC2 19     // Encoder channel C (index/commutation)
#define FG2 27          // Frequency generator (speed feedback)

#define NODE_ROLE "FRONT"
#define DISABLE_CAN 1
#define LED_PIN 13

/* Motor configuration */
#define ENCODER_TICKS_PER_REV 100   // Adjust for your encoder
#define GEAR_RATIO 1.0f             // Adjust for your gearbox
#define FG_PULSES_PER_REV 6         // Typical for brushless motors
#define MAX_RPM 3000.0f             // Maximum motor RPM

// Motor controllers (using A89301 brushless driver interface)
// Constructor: (dirPin, spdPin, brakePin, faultPin, encA, encB, encC, fgPin)
MotorController motor1(DIR1, SPD1, BR1, FLT1, EN_OUTA1, EN_OUTB1, EN_OUTC1, FG1);
MotorController motor2(DIR2, SPD2, BR2, FLT2, EN_OUTA2, EN_OUTB2, EN_OUTC2, FG2);
static unsigned long loopCount = 0;
static unsigned long lastFaultCheck = 0;

// CAN interface
CANInterface canInterface;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000);
  Serial.println("===================================");
  Serial.print("Node Role: ");
  Serial.println(NODE_ROLE);
  Serial.println("===================================");

    /* Initialize Motor 1 */
    if (!motor1.begin()) {
        Serial.println("ERROR: Motor1 initialization failed");
        while (1);
    }
    motor1.setEncoderParams(ENCODER_TICKS_PER_REV, GEAR_RATIO);
    motor1.setFGParams(FG_PULSES_PER_REV);
    motor1.setMaxRPM(MAX_RPM);
    Serial.println("Motor1 initialized successfully");

    /* Initialize Motor 2 */
    if (!motor2.begin()) {
        Serial.println("ERROR: Motor2 initialization failed");
        while (1);
    }
    motor2.setEncoderParams(ENCODER_TICKS_PER_REV, GEAR_RATIO);
    motor2.setFGParams(FG_PULSES_PER_REV);
    motor2.setMaxRPM(MAX_RPM);
    Serial.println("Motor2 initialized successfully");

#if !DISABLE_CAN
    if (!canInterface.begin(NODE_ROLE)) {
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

    /* LED heartbeat */
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
    delay(50);

    /* Check for motor faults (every 100ms) */
    if (millis() - lastFaultCheck > 100) {
        lastFaultCheck = millis();
        
        if (motor1.isFaulted()) {
            BrushlessMotorState state = motor1.getState();
            Serial.print("WARNING: Motor1 fault! State=");
            Serial.println(state);
        }
        
        if (motor2.isFaulted()) {
            BrushlessMotorState state = motor2.getState();
            Serial.print("WARNING: Motor2 fault! State=");
            Serial.println(state);
        }
    }
    // Set test motor speeds
    motor1.setDirection(true);  // Forward
    motor2.setDirection(true); // Forward
    motor1.setSpeed(120.0f);  // 50% speed
    motor2.setSpeed(255.0f);  // 50% speed
    // if (loopCount % 200 == 0) {  // Every ~2 seconds
        
    //     float pwm_speed1 = (loopCount / 200) * 10.0f;
    //     float pwm_speed2 = (loopCount / 200) % 2 == 0 ? -150.0f : 150.0f;
    //     motor1.setSpeedPercent(pwm_speed1 / 255.0f * 100.0f);
    //     motor2.setSpeedPercent(pwm_speed2 / 255.0f * 100.0f);
    //     Serial.print("Set Motor1 speed to ");
    //     Serial.print(pwm_speed1);
    //     Serial.print(" RPM, Motor2 speed to ");
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

    // Use strcmp for robust comparison
    if (strcmp(NODE_ROLE, "FRONT") == 0 && cmd.joint_name.startsWith("F")) {
      if (cmd.joint_name.endsWith("L")) {
        Serial.println("Setting Motor1 speed");
        motor1.setSpeedRPM(cmd.velocity);
      } else if (cmd.joint_name.endsWith("R")) {
        Serial.println("Setting Motor2 speed");
        motor2.setSpeedRPM(cmd.velocity);
      }
    } else if (strcmp(NODE_ROLE, "REAR") == 0 && cmd.joint_name.startsWith("R")) {
      if (cmd.joint_name.endsWith("L")) {
        Serial.println("Setting Motor1 speed");
        motor1.setSpeedRPM(cmd.velocity);
      } else if (cmd.joint_name.endsWith("R")) {
        Serial.println("Setting Motor2 speed");
        motor2.setSpeedRPM(cmd.velocity);
      }
    } else {
      Serial.println("Condition not met for motor control");
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
        float rpm1_enc = motor1.getRPM();
        float rpm1_fg = motor1.getRPMFromFG();
        float rpm2_enc = motor2.getRPM();
        float rpm2_fg = motor2.getRPMFromFG();
        
#if !DISABLE_CAN
        if (strcmp(NODE_ROLE, "FRONT") == 0) {
            canInterface.sendJointFeedback("FL", rpm1_enc);
            canInterface.sendJointFeedback("FR", rpm2_enc);
        } else if (strcmp(NODE_ROLE, "REAR") == 0) {
            canInterface.sendJointFeedback("RL", rpm1_enc);
            canInterface.sendJointFeedback("RR", rpm2_enc);
        }
#endif
        
        /* Debug output every second */
        static unsigned long lastDebug = 0;
        if (millis() - lastDebug > 1000) {
            lastDebug = millis();
            Serial.print("M1 Enc:");
            Serial.print(rpm1_enc);
            Serial.print(" FG:");
            Serial.print(rpm1_fg);
            Serial.print(" | M2 Enc:");
            Serial.print(rpm2_enc);
            Serial.print(" FG:");
            Serial.println(rpm2_fg);
        }
    }
    delay(10);
}