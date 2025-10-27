#include <Arduino.h>
#include "can_interface.h"
#include "motor_controller.h"

#define DIR1 1
#define PWM1 2
#define SLP1 7
#define FLT1 8
#define EN_OUTA1 11
#define EN_OUTB1 12
#define CS1 23

#define DIR2 29
#define PWM2 28
#define SLP2 34
#define FLT2 35
#define EN_OUTA2 24
#define EN_OUTB2 25
#define CS2 40

#define NODE_ROLE "FRONT"
#define DISABLE_CAN 0
#define LED_PIN 13

// Motor controllers
MotorController motor1(DIR1, PWM1, SLP1, FLT1, EN_OUTA1, EN_OUTB1, CS1);
MotorController motor2(DIR2, PWM2, SLP2, FLT2, EN_OUTA2, EN_OUTB2, CS2);

// CAN interface
CANInterface canInterface;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000);
  Serial.println("===================================");
  Serial.print("Node Role: ");
  Serial.println(NODE_ROLE);
  Serial.println("===================================");

  if (!motor1.begin()) {
    Serial.println("ERROR: Motor1 initialization failed");
    while (1);
  }
  if (!motor2.begin()) {
    Serial.println("ERROR: Motor2 initialization failed");
    while (1);
  }

  #if !DISABLE_CAN
  if (!canInterface.begin(NODE_ROLE)) {
    Serial.println("ERROR: CAN initialization failed");
    while (1);
  }
  #else
  Serial.println("CAN disabled (DISABLE_CAN=1)");
  #endif

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  static unsigned long loopCount = 0;
  loopCount++;
  // Serial.print("Loop iteration: ");
  // Serial.println(loopCount);

  digitalWrite(LED_PIN, HIGH);
  delay(50);
  digitalWrite(LED_PIN, LOW);
  delay(50);

  if (digitalRead(motor1.getFaultPin()) == LOW) {
    Serial.println("WARNING: Motor1 fault detected");
  }
  if (digitalRead(motor2.getFaultPin()) == LOW) {
    Serial.println("WARNING: Motor2 fault detected");
  }

  CANJointCommand cmd;
  #if !DISABLE_CAN
  if (canInterface.readJointCommand(cmd)) {
    Serial.print("Received CAN command, joint=");
    Serial.print(cmd.joint_name);
    Serial.print(", velocity=");
    Serial.println(cmd.velocity);
    // Debug string contents
    Serial.print("NODE_ROLE: ");
    Serial.println(NODE_ROLE);
    Serial.print("cmd.joint_name: ");
    Serial.println(cmd.joint_name);

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

  static unsigned long lastFeedback = 0;
  if (millis() - lastFeedback > 1000) { // Increased to 1000ms to reduce bus load
    lastFeedback = millis();
    #if !DISABLE_CAN
    if (strcmp(NODE_ROLE, "FRONT") == 0) {
      float rpm1 = motor1.getRPM();
      float rpm2 = motor2.getRPM();
      Serial.print("Feedback FL, RPM=");
      Serial.println(rpm1);
      canInterface.sendJointFeedback("FL", rpm1);
      Serial.print("Feedback FR, RPM=");
      Serial.println(rpm2);
      canInterface.sendJointFeedback("FR", rpm2);
    } else if (strcmp(NODE_ROLE, "REAR") == 0) {
      float rpm1 = motor1.getRPM();
      float rpm2 = motor2.getRPM();
      Serial.print("Feedback RL, RPM=");
      Serial.println(rpm1);
      canInterface.sendJointFeedback("RL", rpm1);
      Serial.print("Feedback RR, RPM=");
      Serial.println(rpm2);
      canInterface.sendJointFeedback("RR", rpm2);
    }
    #else
    Serial.println("Feedback skipped (CAN disabled)");
    #endif
  }

  delay(10);
}