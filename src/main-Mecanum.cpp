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
MotorController motor1(DIR1, PWM1, SLP1, FLT1, EN_OUTA1, EN_OUTB1, CS1, 1);
MotorController motor2(DIR2, PWM2, SLP2, FLT2, EN_OUTA2, EN_OUTB2, CS2, -1);
MotorController motor3(DIR3, PWM3, SLP3, FLT3, EN_OUTA3, EN_OUTB3, CS3, 1); // Reverse direction for rear motors
MotorController motor4(DIR4, PWM4, SLP4, FLT4, EN_OUTA4, EN_OUTB4, CS4, -1); // Reverse direction for rear motors

MotorController* motors[4] = {&motor1, &motor2, &motor3, &motor4};

MecanumController mecanum(0.15, 0.14, 0.075); // Example wheelbase and trackwidth in meters

// CAN interface
CANInterface canInterface;

#define MESSAGE_TIMEOUT 1500

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

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  digitalWrite(LED_PIN, HIGH);
  delay(50);
  digitalWrite(LED_PIN, LOW);
  delay(50);

  while (!Serial && millis() < 2000);

  int msg_counter = 0;

  SerialAtomics::send(Message::Ping);
  Message msg = Message::Invalid;
  while (msg != Message::Pong) {
    msg = SerialAtomics::recvMsg();

    msg_counter++;
    if (msg_counter == MESSAGE_TIMEOUT) {
      Serial.println("Timed out waiting for arm teensy to come online; starting without it");
      break;
    }
  }

  msg_counter = 0;
  msg = SerialAtomics::recvMsg();
  while (msg != Message::StartGame) {
    msg = SerialAtomics::recvMsg();

    msg_counter++;
    if (msg_counter == MESSAGE_TIMEOUT) {
      Serial.println("Timed out waiting for arm teensy to come online; starting without it");
      break;
    }
  }
}

void loop() {
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

  // === ROTATE TO GOOD START POSITION

  setAllMotorSpeeds(-0.05, -0.05, 0);
  delay(800);
  setAllMotorSpeeds(0, 0, -0.25);
  delay(1900);
  setAllMotorSpeeds(-0.05, 0.05, 0);
  delay(900);

  // === END OF START ROTATION


  // === PRESS THE BIG RED BUTTON ===

  // Forward
  setAllMotorSpeeds(0.2, 0.01, 0);
  delay(1700);

  for (int i = 0; i < 3; i++) {
    // Forward
    setAllMotorSpeeds(0.1, 0.005, 0);
    delay(800);

    // Back
    setAllMotorSpeeds(-0.1, 0.005, 0);
    delay(600);
  }

  // stop & move arm to read
  setAllMotorSpeeds(0, 0, 0);
  SerialAtomics::send(Message::MoveArm);
  SerialAtomics::send(ArmPositions::READ_COLOR);
  delay(500);

  // tad bit forward
  setAllMotorSpeeds(0.05, 0, 0);
  delay(1800);

  // pause for read
  SerialAtomics::send(Message::ReadThenSetLED);
  SerialAtomics::send(0);
  setAllMotorSpeeds(0, 0, 0);
  delay(4000);

  // back up before sideways
  setAllMotorSpeeds(-0.1, 0.005, 0);
  delay(800);

  // Arm down
  SerialAtomics::send(Message::MoveArm);
  SerialAtomics::send(ArmPositions::COLLAPSED);

  /// === END OF BUTTON


  // === SPIN THAT KNOB ===

  // Drive Right to push duck
  setAllMotorSpeeds(0, -0.2, 0);
  delay(1800);

  // back up and away from duck
  setAllMotorSpeeds(0, 0.1, 0);
  delay(500);
  setAllMotorSpeeds(-0.1, 0, 0);
  delay(1000);

  //rotate
  setAllMotorSpeeds(0, 0, -0.25);
  delay(2000);

  // Move to crater
  setAllMotorSpeeds(0.2, 0, 0);
  delay(1800);

  //hit far wall 
  setAllMotorSpeeds(0, 0.1, 0);
  delay(2800);

  //go towards knob along wall
  setAllMotorSpeeds(0.2, 0.005, 0);
  delay(2500);

  //back up a bit
  setAllMotorSpeeds(-0.1, 0, 0);
  delay(400);
  setAllMotorSpeeds(0, -0.1, 0);
  delay(800);

  //rotate -90 degrees
  setAllMotorSpeeds(0, 0, -0.25);
  delay(1800);

  //move to far wall, right then forward
  setAllMotorSpeeds(0.1, 0.1, 0);
  delay(1200);
  setAllMotorSpeeds(0, 0.1, 0);
  delay(1000);

  //push duck out of way
  setAllMotorSpeeds(0.2, 0.005, 0);
  delay(800);
  setAllMotorSpeeds(-0.2, 0.005, 0);
  delay(800);

  // drive to & spin knob
  SerialAtomics::send(Message::StartKnob);
  setAllMotorSpeeds(-0.05, 0.005, 0);
  delay(2000);
  setAllMotorSpeeds(-0.01, 0.01, 0);

  // wait for knob to finish
  delay(2000);
  SerialAtomics::send(Message::StopKnob);

  //back off knob
  setAllMotorSpeeds(0.05, -0.05, 0);
  delay(1000);

  //rotate little less than 180 to read
  setAllMotorSpeeds(0, 0, 0.25);
  delay(3200);

  // move arm into reading pos & get closer
  SerialAtomics::send(Message::MoveArm);
  SerialAtomics::send(ArmPositions::READ_COLOR);
  setAllMotorSpeeds(0.05, 0, 0);
  delay(1300);

  // align w/ antenna
  setAllMotorSpeeds(0, -0.05, 0);
  delay(1500);

  // scooch forward
  setAllMotorSpeeds(0.05, 0, 0);
  delay(600);

  //pause for read
  SerialAtomics::send(Message::ReadThenSetLED);
  SerialAtomics::send(1);
  setAllMotorSpeeds(0, 0, 0);
  delay(4000);

  //back off antenna
  setAllMotorSpeeds(-0.1, .05, 0);
  delay(900);

  // === END OF KNOB ===


  // === HELLDIVERS ===

  // move towards crater
  setAllMotorSpeeds(0, 0.1, 0);
  delay(800);

  // slam into wall behind spinny thing antenna
  setAllMotorSpeeds(0.15, 0.05, 0);
  delay(2000);

  // move left to where helldivers should be deployed
  setAllMotorSpeeds(0.005, 0.1, 0);
  delay(1500);

  // Stop and deploy HELLDIVERS
  setAllMotorSpeeds(0, 0, 0);

  // DEPLOY THE HELLDIVERS
  SerialAtomics::send(Message::MoveBugs);
  SerialAtomics::send(BugPositions::HELLDIVE);
  delay(2000);
  SerialAtomics::send(Message::MoveBugs);
  SerialAtomics::send(BugPositions::COLLAPSED);
  delay(1000);
  SerialAtomics::send(Message::MoveArm);
  SerialAtomics::send(ArmPositions::COLLAPSED);

  // continue moving past helldivers, bump button, back off
  setAllMotorSpeeds(0.005, 0.1, 0);
  delay(7500);
  setAllMotorSpeeds(0.005, -0.1, 0);
  delay(1000);

  // back off wall
  setAllMotorSpeeds(-0.15, 0, 0);
  delay(1500);

  //rotate 90 degrees
  setAllMotorSpeeds(0, 0, 0.25);
  delay(1800);

  // go to start corner
  setAllMotorSpeeds(0.1, 0.1, 0);
  delay(3200);
  setAllMotorSpeeds(0.05, 0.05, 0);
  delay(300);
  //Start of corner to keypad test

  // === END OF HELLDIVERS ===


  // --- PUSH DUCK #2

  //bump keypad antenna
  setAllMotorSpeeds(-0.1, 0.008, 0);
  delay(3500);
  setAllMotorSpeeds(-0.05, 0, 0); //slow down
  delay(1000);

  //back off a bit
  setAllMotorSpeeds(0.1, 0.005, 0);
  delay(500);
  
  //push duck forward
  setAllMotorSpeeds(0, -0.1, 0);
  delay(3000);

  //return to home
  setAllMotorSpeeds(0.08, 0.08, 0);
  delay(6000);

  // === END OF PUSH DUCK #2


  // === PUSH KEYPAD ===

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
  delay(1900);

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
  setAllMotorSpeeds(0.05, -0.05, 0);
  delay(900);

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
  SerialAtomics::send(Message::StartSolenoids);
  delay(10000);
  SerialAtomics::send(Message::StopSolenoids);

  // === END OF PUSH KEYPAD


  // === READ KEYPAD

  //back off keypads
  setAllMotorSpeeds(0, 0.1, 0);
  delay(500);

  //rotate -90
  setAllMotorSpeeds(0, 0, -0.25);
  delay(1800);

  // stop & move arm to read
  setAllMotorSpeeds(0, 0, 0);
  SerialAtomics::send(Message::MoveArm);
  SerialAtomics::send(ArmPositions::READ_COLOR);
  delay(1000);

  // push in
  setAllMotorSpeeds(0.05, -0.0025, 0);
  delay(1800);

  // pause for read
  SerialAtomics::send(Message::ReadThenSetLED);
  SerialAtomics::send(2);
  setAllMotorSpeeds(0, 0, 0);
  delay(5000);

  // === END OF READ KEYPAD ===



  // === MOVE TO CRATER ===

  //back off
  setAllMotorSpeeds(-0.1, 0, 0);
  delay(1200);

  //collapse arm
  setAllMotorSpeeds(0, 0, 0);
  SerialAtomics::send(Message::MoveArm);
  SerialAtomics::send(ArmPositions::COLLAPSED); 
  delay(1000); 

  //rotate 90
  setAllMotorSpeeds(0, 0, 0.25);
  delay(1800);

  //bump button wall
  setAllMotorSpeeds(0.05, 0.1, 0);
  delay(3000);

  //bump button keypad
  setAllMotorSpeeds(-0.1, 0.025, 0);
  delay(6000);

  //go to crater position
  setAllMotorSpeeds(0.1, 0.025, 0);
  delay(4500);

  // === KNOCK ANTENNA DUCK ===
  
  //put arm in duck position
  setAllMotorSpeeds(0, 0, 0);
  SerialAtomics::send(Message::MoveArm);
  SerialAtomics::send(ArmPositions::KNOCK_DUCK); 
  delay(1500);

  //swipe duck
  setAllMotorSpeeds(0, -0.1, 0);
  delay(3500);
  
  //read crater antenna
  setAllMotorSpeeds(0, 0, 0);
  SerialAtomics::send(Message::MoveArm);
  SerialAtomics::send(ArmPositions::ANT_READ_COLOR); 
  delay(1000);

  //go to read position
  setAllMotorSpeeds(-0.05, 0, 0);
  delay(800);
  setAllMotorSpeeds(0, 0.05, 0);
  delay(2000);
  setAllMotorSpeeds(0.05, 0, 0);
  delay(1000);

  // pause for read
  setAllMotorSpeeds(0, 0, 0);
  SerialAtomics::send(Message::ReadThenSetLED);
  SerialAtomics::send(3);
  delay(5000);

  //back off
  setAllMotorSpeeds(-0.1, 0, 0);
  delay(2000);
  setAllMotorSpeeds(0, 0, 0);
  SerialAtomics::send(Message::MoveArm);
  SerialAtomics::send(ArmPositions::KNOCK_DUCK); 
  delay(1000);
  SerialAtomics::send(Message::MoveArm);
  SerialAtomics::send(ArmPositions::COLLAPSED); 


  // === END Of KNOCK ANTENNA DUCK ===

  // === PUSH DUCKS

  setAllMotorSpeeds(-0.1, 0, 0);
  delay(3000);
  setAllMotorSpeeds(0, 0.1, 0);
  delay(1000);
  setAllMotorSpeeds(0.1, 0, 0);
  delay(2000);

  // === ET PHONE HOME ===

  setAllMotorSpeeds(-0.07, -0.05, 0);
  delay(5000);

  // -- STOP ALL MOTORS --
  setAllMotorSpeeds(0, 0, 0);
  while (true) {
    
  }
}
