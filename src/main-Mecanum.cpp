#include <Arduino.h>
#include "can_interface.h"
#include "motor_controller.h"
#include "mecanum_controller.h"
#include <timer.h>
#include <TimeLib.h>
#include <SerialAtomics.h>
#include <Arm.h>

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
  SerialAtomics::setup();

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

  SerialAtomics::send(Message::Ping);
  Message msg = Message::Invalid;
  while (msg != Message::Pong) {
    msg = SerialAtomics::recvMsg();
  }

  msg = SerialAtomics::recvMsg();
  while (msg != Message::StartGame) {
    msg = SerialAtomics::recvMsg();
  }
}

void loop() {
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


  // === PRESS THE BIG RED BUTTON ===

  // Forward
  setAllMotorSpeeds(0.2, 0.01, 0);
  delay(1800);

  for (int i = 0; i < 3; i++) {
    // Forward
    setAllMotorSpeeds(0.1, 0.05, 0);
    delay(800);

    // Back
    setAllMotorSpeeds(-0.1, 0.05, 0);
    delay(600);
  }

  // stop & move arm to read
  setAllMotorSpeeds(0, 0, 0);
  SerialAtomics::send(Message::MoveArm);
  SerialAtomics::send(ArmPositions::READ_COLOR);
  delay(1000);

  // pause for read
  SerialAtomics::send(Message::ReadThenSetLED);
  SerialAtomics::send(0);
  setAllMotorSpeeds(0, 0, 0);
  delay(5000);

  // Arm down
  SerialAtomics::send(Message::MoveArm);
  SerialAtomics::send(ArmPositions::COLLAPSED);


  // === SPIN THAT KNOB ===
  

  // Drive Right to push duck
  setAllMotorSpeeds(0, -0.1, 0);
  delay(2700);

  // back up and away from duck
  setAllMotorSpeeds(0, 0.1, 0);
  delay(500);
  setAllMotorSpeeds(-0.1, 0, 0);
  delay(1000);

  //rotate
  setAllMotorSpeeds(0, 0, -0.25);
  delay(2000);

  // Move to crater
  setAllMotorSpeeds(0.1, 0, 0);
  delay(2500);

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
  delay(1000);

  //rotate 180 to read
  setAllMotorSpeeds(0, 0, 0.25);
  delay(3400);

  // move arm into reading pos & get closer
  SerialAtomics::send(Message::MoveArm);
  SerialAtomics::send(ArmPositions::READ_COLOR);
  setAllMotorSpeeds(.05, 0, 0);
  delay(1000);

  // align w/ antenna
  setAllMotorSpeeds(0, -0.05, 0);
  delay(1500);

  //pause for read
  SerialAtomics::send(Message::ReadThenSetLED);
  SerialAtomics::send(1);
  setAllMotorSpeeds(0, 0, 0);
  delay(5000);

  //back off antenna
  setAllMotorSpeeds(-0.1, .05, 0);
  delay(950);


  // === HELLDIVERS ===


  // move towards crater
  setAllMotorSpeeds(0, 0.1, 0);
  delay(800);

  // slam into wall behind spinny thing antenna
  setAllMotorSpeeds(0.15, 0, 0);
  delay(2000);

  // move left to where helldivers should be deployed
  setAllMotorSpeeds(0, 0.1, 0);
  delay(800);

  // DEPLOY THE HELLDIVERS
  SerialAtomics::send(Message::MoveBugs);
  SerialAtomics::send(BugPositions::HELLDIVE);
  delay(3000);
  SerialAtomics::send(Message::MoveBugs);
  SerialAtomics::send(BugPositions::COLLAPSED);
  delay(1000);
  SerialAtomics::send(Message::MoveArm);
  SerialAtomics::send(ArmPositions::COLLAPSED);
  delay(1000);

  while (true) {}


  // === BACK TO START ===


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
  

  // === SCORE SECOND DUCK ===


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


  // === KEYPAD ===


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
  delay(300);

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

  // wait for a few keypad attempts
  delay(5000);

  // back off wall
  setAllMotorSpeeds(0, 0.1, 0);
  delay(1000);

  
  // === DA BOWL ===
  

  // knock duck off middle station
  SerialAtomics::send(Message::MoveArm);
  SerialAtomics::send(ArmPositions::READ_COLOR);
  delay(500);
  SerialAtomics::send(Message::MoveArm);
  SerialAtomics::send(ArmPositions::DOWN);
  delay(500);
  SerialAtomics::send(Message::MoveArm);
  SerialAtomics::send(ArmPositions::KNOCK_DUCK);
  delay(500);

  // drive forwards to make sure we knock duck off
  setAllMotorSpeeds(0.05, 0, 0);
  delay(1000);

  // -- STOP ALL MOTORS --
  setAllMotorSpeeds(0, 0, 0);
  while (true) {
    
  }
}
