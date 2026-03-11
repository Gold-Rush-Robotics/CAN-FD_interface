#include <Arduino.h>
#include "can_interface.h"
#include "motor_controller.h"
#include "mecanum_controller.h"
#include <timer.h>

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
MotorController motor2(DIR2, PWM2, SLP2, FLT2, EN_OUTA2, EN_OUTB2, CS2, -1);
MotorController motor3(DIR3, PWM3, SLP3, FLT3, EN_OUTA3, EN_OUTB3, CS3); // Reverse direction for rear motors
MotorController motor4(DIR4, PWM4, SLP4, FLT4, EN_OUTA4, EN_OUTB4, CS4, -1); // Reverse direction for rear motors

MotorController* motors[4] = {&motor1, &motor2, &motor3, &motor4};

MecanumController mecanum(0.15, 0.14, 0.075); // Example wheelbase and trackwidth in meters

// CAN interface
CANInterface canInterface;

void setAllMotorSpeeds(float linear_x, float linear_y, float angular_z) {
  float* wheelSpeeds = mecanum.calculateMecanumWheelSpeeds(-linear_x, linear_y, angular_z);
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
  Timer timer = Timer();


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
  setAllMotorSpeeds(0.1, 0.005, 0);
  delay(4200);

  // Back
  setAllMotorSpeeds(-0.1, 0.05, 0);
  delay(600);

  // Forward
  setAllMotorSpeeds(0.1, 0.05, 0);
  delay(800);

  // Back
  setAllMotorSpeeds(-0.1, 0.05, 0);
  delay(600);
  setAllMotorSpeeds(0, 0, 0);

  // align with button antenna
  timer.waitUntil(9000);
  setAllMotorSpeeds(0.1, -0.06, 0.09);
  delay(900);
  setAllMotorSpeeds(-0.05, 0, 0);
  delay(200); 

  //pause for read
  setAllMotorSpeeds(0, 0, 0); //pause for read
  delay(1000);

  // Back
  setAllMotorSpeeds(-0.1, 0, 0);
  delay(600);  

  //exit sync space

  // -- GO TO SPINNY THING --
  //Bump wall to square
  setAllMotorSpeeds(0, 0.1, 0);
  delay(1200);

  //Bump button to square
  setAllMotorSpeeds(0.1, 0, 0);
  delay(1000);
  setAllMotorSpeeds(-0.1, 0, 0);
  delay(600);

  //Drive Right to push duck
  setAllMotorSpeeds(0, -0.1, 0);
  delay(2700);

  setAllMotorSpeeds(0, 0.1, 0);
  delay(500);

  setAllMotorSpeeds(-0.1, 0, 0);
  delay(1000);

  //rotate
  setAllMotorSpeeds(0, 0, -0.25);
  delay(2000);

  // This gets close to crater edge
  setAllMotorSpeeds(0.1, 0, 0);
  delay(2200);

  //hit far wall 
  setAllMotorSpeeds(0, 0.1, 0);
  delay(2900);

  //go towards knob along wall
  setAllMotorSpeeds(0.1, 0.005, 0);
  delay(5800);

  //back up a bit
  setAllMotorSpeeds(-0.1, 0, 0);
  delay(400);
  setAllMotorSpeeds(0, -0.1, 0);
  delay(800);

  //rotate -90 degrees
  setAllMotorSpeeds(0, 0, -0.25);
  delay(1800);

  //move to far wall, right then forward
  setAllMotorSpeeds(0.05, 0.05, 0);
  delay(1500);
  setAllMotorSpeeds(0, 0.1, 0);
  delay(1200);
  

  //run to knob
  setAllMotorSpeeds(-0.05, 0.005, 0);
  delay(1700);
  setAllMotorSpeeds(-0.005, 0.005, 0);
  delay(5000);

  //back off knob
  setAllMotorSpeeds(0.05, -0.05, 0);
  delay(900);

  //rotate 180 + 45 to read
  setAllMotorSpeeds(0, 0, 0.2);
  delay(1800 * 2);
  setAllMotorSpeeds(0, 0, 0);

  //align with antenna
  timer.waitUntil(50000);
  setAllMotorSpeeds(0.05, 0, 0);
  delay(1100);
  setAllMotorSpeeds(0, 0.05, 0);
  delay(1200);
  setAllMotorSpeeds(0.05, 0, 0);
  delay(300);
  

  //pause for read
  setAllMotorSpeeds(0, 0, 0);
  delay(2000);

  //back off antenna
  setAllMotorSpeeds(-0.1, 0, 0);
  delay(950);

  //align with crater
  setAllMotorSpeeds(0, 0.1, 0);
  delay(1300);
  setAllMotorSpeeds(0, 0, 0);

  //get here at ~57000, wait 5s for deploy
  timer.waitUntil(57000 + 5000);

  //bump against long wall
  setAllMotorSpeeds(0, 0.1, -0.1);
  delay(2000);


  // --- MOVE BACK TO START

  //move back towards button along long wall
  setAllMotorSpeeds(-0.1, 0.005, 0);
  delay(4000);

  //back off long wall, towards keypad
  setAllMotorSpeeds(0, -0.1, 0);
  delay(2000);
  
  //rotate 90 degrees
  setAllMotorSpeeds(0, 0, 0.25);
  delay(1800);

  //go towards short wall
  setAllMotorSpeeds(0, 0.1, 0);
  delay(5500);

  //back off wall
  setAllMotorSpeeds(0, -0.1, 0);
  delay(600);
  
  //rotate 90 degrees
  setAllMotorSpeeds(0, 0, 0.25);
  delay(1800);

  //go into start square
  setAllMotorSpeeds(0.04, 0.1, 0);
  delay(3000);
  //Perfectly Square
  

  

  // --- PUSH DUCK #2

  //bump keypad antenna
  setAllMotorSpeeds(-0.1, 0.005, 0);
  delay(3500);
  setAllMotorSpeeds(-0.05, 0, 0);
  delay(1000);

  //push duck and return 
  setAllMotorSpeeds(0, -0.1, 0);
  delay(3500);
  setAllMotorSpeeds(0.05, 0.05, 0);
  delay(9000);

  // --- END OF PUSH DUCK #2



  // --- KEYPAD SOLENOIDS

  //back out and rotate -90
  setAllMotorSpeeds(-0.05, -0.05, 0);
  delay(1000);
  setAllMotorSpeeds(0, 0, -0.25);
  delay(1800);

  //bump short wall
  setAllMotorSpeeds(0, 0.1, 0);
  delay(600);

  //bump long wall
  setAllMotorSpeeds(-0.1, 0, 0);
  delay(700);

  //back out and rotate -90
  setAllMotorSpeeds(0.05, -0.05, 0);
  delay(1000);
  setAllMotorSpeeds(0, 0, -0.25);
  delay(1800);

  //bump solenoids and back off a smidge
  setAllMotorSpeeds(0, -0.05, 0);
  delay(1500);
  setAllMotorSpeeds(0, 0.05, 0);
  delay(100);

  //bump keypad
  setAllMotorSpeeds(0.1, 0, 0);
  delay(3000);
  setAllMotorSpeeds(0.05, 0, 0); //slow down
  delay(1500);

  //bump solenoids again
  setAllMotorSpeeds(0, -0.05, 0);
  delay(600);

  //back off keypad
  setAllMotorSpeeds(-0.1, 0, 0);
  delay(300);

  //back off wall
  setAllMotorSpeeds(0, 0.1, 0);
  delay(1200);

  //align with keypad
  setAllMotorSpeeds(0.05, 0, 0); //drive over
  delay(2000);
  setAllMotorSpeeds(0.05, -0.02, 0); //drive up to keys
  delay(1500);
  setAllMotorSpeeds(0, -0.05, 0); //provide force into it
  delay(200);










  //Stop
  setAllMotorSpeeds(0, 0, 0);
  delay(10000000);

  // Push duck into blue
  // setAllMotorSpeeds(0, 0.25, 0);
  // delay(300);

  // setAllMotorSpeeds(0.2, 0, 0);
  // delay(1800);

  // setAllMotorSpeeds(0.05, 0.25, 0);
  // delay(4100);

  // setAllMotorSpeeds(0.2, 0, 0);
  // delay(800);

  // setAllMotorSpeeds(0.1, -0.25, 0);
  // delay(300);

  // setAllMotorSpeeds(0.2, 0, 0);
  // delay(400);
  
  // setAllMotorSpeeds(-0.2, 0.05, 0);
  // delay(600);

  /*
  // TURN
  setAllMotorSpeeds(0, 0, 0.5);
  delay(1500);

  setAllMotorSpeeds(0.2, 0, 0);
  delay(1100);

  setAllMotorSpeeds(-0.1, 0, 0);
  delay(200);

  setAllMotorSpeeds(0, -0.125, 0);
  delay(3000);

  setAllMotorSpeeds(0.1, -0.1, 0);
  delay(3000);

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

  setAllMotorSpeeds(0.2, -0.1, 0);
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

  // 3 attempts at hitting keypad
  for (int i = 0; i < 3; i++) {
    // Push into keypad
    setAllMotorSpeeds(-0.025, 0.025, 0);
    delay(12000); // align + 2-3 attempts
  
    // Go back for realignment
    setAllMotorSpeeds(0.1, -0.12, 0);
    delay(500);
  }

  // -- PUSH DUCK TO BLUE SQUARE --

  // Align to duck
  setAllMotorSpeeds(0.05, -0.15, 0);
  delay(700);

  // Push duck into blue square
  setAllMotorSpeeds(0.15, 0.075, 0);
  delay(1400);

  // -- GO HOME --
  setAllMotorSpeeds(-0.15, 0, 0);
  delay(600);

  setAllMotorSpeeds(-0.05, -0.15, 0);
  delay(4000);

  setAllMotorSpeeds(-0.15, -0.05, 0);
  delay(4000);

  */


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
