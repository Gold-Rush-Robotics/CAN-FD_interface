#include <Arduino.h>
#include "can_interface.h"
#include "motor_controller.h"
#include "mecanum_controller.h"

#define DIR1 4
#define PWM1 3
#define SLP1 2
#define FLT1 1
#define EN_OUTA1 5
#define EN_OUTB1 6
#define CS1 0

#define DIR2 11
#define PWM2 10
#define SLP2 9
#define FLT2 8
#define EN_OUTA2 12
#define EN_OUTB2 24
#define CS2 7

#define DIR3 21
#define PWM3 19
#define SLP3 18
#define FLT3 17
#define EN_OUTA3 22
#define EN_OUTB3 23
#define CS3 16

#define DIR4 41
#define PWM4 13
#define SLP4 40
#define FLT4 39
#define EN_OUTA4 14
#define EN_OUTB4 15
#define CS4 38

#define NODE_ROLE "FRONT"
#define DISABLE_CAN 1
#define LED_PIN 13

// Motor controllers
MotorController motor1(DIR1, PWM1, SLP1, FLT1, EN_OUTA1, EN_OUTB1, CS1);
MotorController motor2(DIR2, PWM2, SLP2, FLT2, EN_OUTA2, EN_OUTB2, CS2);
MotorController motor3(DIR3, PWM3, SLP3, FLT3, EN_OUTA3, EN_OUTB3, CS3, -1); // Reverse direction for rear motors
MotorController motor4(DIR4, PWM4, SLP4, FLT4, EN_OUTA4, EN_OUTB4, CS4, -1); // Reverse direction for rear motors

MotorController* motors[4] = {&motor1, &motor2, &motor3, &motor4};

MecanumController mecanum(0.15, 0.14, 0.075); // Example wheelbase and trackwidth in meters

// CAN interface
CANInterface canInterface;

void setAllMotorSpeeds(float linear_x, float linear_y, float angular_z) {
  float* wheelSpeeds = mecanum.calculateMecanumWheelSpeeds(linear_x, linear_y, angular_z);
  Serial.print("Wheel speeds: ");
  for (int i = 0; i < 4; i++) {
    Serial.print(wheelSpeeds[i]);
    motors[i]->setSpeedRPM(wheelSpeeds[i] * 30.0/1.6);

    Serial.print(" ");
  }
  Serial.println();
}

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
    Serial.println("ERROR: Motdor2 initialization failed");
    while (1);
  }
  if (!motor3.begin()) {
    Serial.println("ERROR: Motor3 initialization failed");
    while (1);
  }
  if (!motor4.begin()) {
    Serial.println("ERROR: Motor4 initialization failed");
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
  if (digitalRead(motor3.getFaultPin()) == LOW) {
    Serial.println("WARNING: Motor3 fault detected");
  }
  if (digitalRead(motor4.getFaultPin()) == LOW) {
    Serial.println("WARNING: Motor4 fault detected");
  }

  // -- PRESS BUTTON 3 TIMES AND GO BACK --
  // Forward
  setAllMotorSpeeds(0.2, 0, 0.0);
  delay(2300);

  // Back
  setAllMotorSpeeds(-0.2, 0, 0.0);
  delay(600);

  // Forward
  setAllMotorSpeeds(0.2, 0, 0.0);
  delay(800);

  // Back
  setAllMotorSpeeds(-0.2, 0, 0.0);
  delay(600);

  // Forward
  setAllMotorSpeeds(0.2, 0, 0.0);
  delay(800);

  // -- GO TO SPINNY THING --
  setAllMotorSpeeds(-0.05, 0.1, 0.0);
  delay(1800);

  // Push duck into blue
  setAllMotorSpeeds(0, 0.25, 0);
  delay(300);

  setAllMotorSpeeds(0.2, 0, 0);
  delay(1800);

  setAllMotorSpeeds(0.05, 0.25, 0);
  delay(4100);

  setAllMotorSpeeds(0.1, -0.25, 0);
  delay(220);

  setAllMotorSpeeds(0.2, 0, 0);
  delay(400);
  
  setAllMotorSpeeds(-0.2, 0.075, 0);
  delay(600);

  // TURN
  setAllMotorSpeeds(0, 0, 0.5);
  delay(1500);

  setAllMotorSpeeds(0.2, 0, 0);
  delay(1000);

  setAllMotorSpeeds(-0.1, 0, 0);
  delay(200);

  setAllMotorSpeeds(0, -0.125, 0);
  delay(3000);

  setAllMotorSpeeds(0.1, -0.1, 0);
  delay(7000);

  // -- GO TO KEYPAD --
  setAllMotorSpeeds(-0.1, 0.1, 0);
  delay(200);

  setAllMotorSpeeds(0, 0.2, 0);
  delay(500);

  setAllMotorSpeeds(0.2, 0, 0);
  delay(500);

  setAllMotorSpeeds(-0.2, 0, 0);
  delay(700);

  // Turn
  setAllMotorSpeeds(0, 0, -0.5);
  delay(1600);

  setAllMotorSpeeds(0.2, -0.0375, 0);
  delay(1600);

  // Slam into button antenna
  setAllMotorSpeeds(0.05, -0.25, 0);
  delay(3800);

  setAllMotorSpeeds(0.1, 0.1, 0);
  delay(300);

  // Hit back wall
  setAllMotorSpeeds(-0.15, 0, 0);
  delay(3500);

  setAllMotorSpeeds(0.2, 0, 0);
  delay(100);
  
  // Hit keypad antenna
  setAllMotorSpeeds(0, 0.15, 0);
  delay(1800);
  
  setAllMotorSpeeds(0, -0.15, 0);
  delay(100);

  setAllMotorSpeeds(-0.15, 0, 0);
  delay(200);

  setAllMotorSpeeds(0.15, 0, 0);
  delay(800);

  setAllMotorSpeeds(0, 0.15, 0);
  delay(550);

  // Push into keypad
  setAllMotorSpeeds(-0.025, 0.025, 0);
  delay(30000);

  // -- PUSH DUCK TO BLUE SQUARE --

  // Align to duck
  setAllMotorSpeeds(0.05, -0.15, 0);
  delay(700);

  // Push duck into blue square
  setAllMotorSpeeds(0.15, 0.075, 0);
  delay(1600);

  // -- GO HOME --
  setAllMotorSpeeds(-0.15, 0, 0);
  delay(600);

  setAllMotorSpeeds(-0.05, -0.15, 0);
  delay(4000);

  setAllMotorSpeeds(-0.15, -0.05, 0);
  delay(4000);


  // -- STOP ALL MOTORS --
  setAllMotorSpeeds(0, 0, 0);
  while (true) {
    
  }

  // setAllMotorSpeeds(0.0, 0.5, 0.0); // Example: move sidewards at half speed
  // delay(5000);

  // setAllMotorSpeeds(0.0, 0.0, 0.5); // Example: rotate clockwise at half speed
  // delay(5000);


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
        Serial.println("Setting Motor3 speed");
        motor3.setSpeedRPM(cmd.velocity);
      } else if (cmd.joint_name.endsWith("R")) {
        Serial.println("Setting Motor4 speed");
        motor4.setSpeedRPM(cmd.velocity);
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
      float rpm3 = motor3.getRPM();
      float rpm4 = motor4.getRPM();
      Serial.print("Feedback FL, RPM=");
      Serial.println(rpm1);
      canInterface.sendJointFeedback("FL", rpm1);
      Serial.print("Feedback FR, RPM=");
      Serial.println(rpm2);
      canInterface.sendJointFeedback("FR", rpm2);
      Serial.print("Feedback RL, RPM=");
      Serial.println(rpm3);
      canInterface.sendJointFeedback("RL", rpm3);
      Serial.print("Feedback RR, RPM=");
      Serial.println(rpm4);
      canInterface.sendJointFeedback("RR", rpm4);
    } else if (strcmp(NODE_ROLE, "REAR") == 0) {
      float rpm1 = motor3.getRPM();
      float rpm2 = motor4.getRPM();
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