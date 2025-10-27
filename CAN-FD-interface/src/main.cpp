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

// Pin definitions for Motor 2
#define DIR2 29
#define PWM2 28
#define SLP2 34
#define FLT2 35
#define EN_OUTA2 24
#define EN_OUTB2 25
#define CS2 40

#define NODE_ROLE "FRONT"

// Motor controllers
MotorController motor1(DIR1, PWM1, SLP1, FLT1, EN_OUTA1, EN_OUTB1, CS1);
MotorController motor2(DIR2, PWM2, SLP2, FLT2, EN_OUTA2, EN_OUTB2, CS2);

// CAN interface
CANInterface canInterface;

//led setup
#define LED_PIN 13


void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("===================================");
  Serial.print("Node Role: ");
  Serial.println(NODE_ROLE);
  Serial.println("===================================");

  motor1.begin();
  motor2.begin();

  // Assign CAN IDs based on role
  canInterface.begin(NODE_ROLE);
}

void loop() {
  //blink led to show alive
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
  delay(100);
  Serial.println("Main loop iteration");
  // linmit loop speed for serial monitor readability
  delay(100);
  Serial.println("Main loop iteration");
  CANJointCommand cmd;

  // Read incoming CAN commands
  if (canInterface.readJointCommand(cmd)) {
    if (String(NODE_ROLE) == "FRONT" && cmd.joint_name.startsWith("F")) {
      if (cmd.joint_name.endsWith("L")) motor1.setSpeedRPM(cmd.velocity);
      else if (cmd.joint_name.endsWith("R")) motor2.setSpeedRPM(cmd.velocity);
    }
    else if (String(NODE_ROLE) == "REAR" && cmd.joint_name.startsWith("R")) {
      if (cmd.joint_name.endsWith("L")) motor1.setSpeedRPM(cmd.velocity);
      else if (cmd.joint_name.endsWith("R")) motor2.setSpeedRPM(cmd.velocity);
    }
  }
  Serial.println("Main loop iteration");
  // Send periodic heartbeat
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 1000) {
    lastHeartbeat = millis();
    bool hb_sent = canInterface.sendHeartbeat();
    if (hb_sent) {
      Serial.println("Heartbeat sent");
    } else {
      Serial.println("Failed to send Heartbeat");
    }
  }
  Serial.println("Main loop iteration");

  // Periodic feedback
  static unsigned long lastFeedback = 0;
  if (millis() - lastFeedback > 500) {
    lastFeedback = millis();
    if (String(NODE_ROLE) == "FRONT") {
      canInterface.sendJointFeedback("FL", motor1.getRPM());
      canInterface.sendJointFeedback("FR", motor2.getRPM());
    } else if (String(NODE_ROLE) == "REAR") {
      canInterface.sendJointFeedback("RL", motor1.getRPM());
      canInterface.sendJointFeedback("RR", motor2.getRPM());
    }
  }
  Serial.println("Main loop iteration");

  // delay(10);
  Serial.println("Main loop iteration");
}