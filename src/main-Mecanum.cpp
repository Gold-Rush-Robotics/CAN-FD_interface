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

double Kp = 0.76;
double Ki = 1.5;
double Kd = 0.00;
double POn = 1.5;

// Motor controllers
MotorController motor1(DIR1, PWM1, SLP1, FLT1, EN_OUTA1, EN_OUTB1, CS1, 1, Kp, Ki, Kd, POn);
MotorController motor2(DIR2, PWM2, SLP2, FLT2, EN_OUTA2, EN_OUTB2, CS2, -1, Kp, Ki, Kd, POn);
MotorController motor3(DIR3, PWM3, SLP3, FLT3, EN_OUTA3, EN_OUTB3, CS3, 1, Kp, Ki, Kd, POn); // Reverse direction for rear motors
MotorController motor4(DIR4, PWM4, SLP4, FLT4, EN_OUTA4, EN_OUTB4, CS4, -1, Kp, Ki, Kd, POn); // Reverse direction for rear motors

MotorController* motors[4] = {&motor1, &motor2, &motor3, &motor4};

MecanumController mecanum(0.15, 0.14, 0.075); // Example wheelbase and trackwidth in meters

void PidDelay(int ms) {
  unsigned long startTime = millis();
  while(millis() < startTime + ms) {
    for(int i = 0; i < 4; i++) {
      motors[i]->PidLoop();
    }
    delay(10);
    for(int i = 0; i < 4; i++) {
        Serial.print("SP: ");
        Serial.print(motors[i]->getSetpoint());
        Serial.print(" RPM: ");
        Serial.print(motors[i]->getRPM());
        Serial.print(" OUT: ");
        Serial.println(motors[i]->getOutput());
    }
  }
}

// CAN interface
CANInterface canInterface;

void setAllMotorSpeeds(float linear_x, float linear_y, float angular_z) {
  float* wheelSpeeds = mecanum.calculateMecanumWheelSpeeds(-linear_x, linear_y, angular_z);
  Serial.print("Wheel speeds: ");
  for (int i = 0; i < 4; i++) {
    Serial.print(wheelSpeeds[i]);
    motors[i]->setSetpoint(wheelSpeeds[i] * 30.0/1.6);

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
    Serial.println("ERROR: Motor2 initialization failed");
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
  //Start of PID and PID Delay
  PidDelay(1800);

  for (int i = 0; i < 3; i++) {
    // Forward
    setAllMotorSpeeds(0.1, 0.05, 0);
    PidDelay(800);

    // Back
    setAllMotorSpeeds(-0.1, 0.05, 0);
    PidDelay(600);
  }

  // stop & move arm to read
  setAllMotorSpeeds(0, 0, 0);
  SerialAtomics::send(Message::MoveArm);
  SerialAtomics::send(ArmPositions::READ_COLOR);
  PidDelay(1000);

  // tad bit forward
  setAllMotorSpeeds(0.05, 0, 0);
  PidDelay(500);

  // pause for read
  SerialAtomics::send(Message::ReadThenSetLED);
  SerialAtomics::send(0);
  setAllMotorSpeeds(0, 0, 0);
  PidDelay(5000);

  // Arm down
  SerialAtomics::send(Message::MoveArm);
  SerialAtomics::send(ArmPositions::COLLAPSED);


  // === SPIN THAT KNOB ===
  

  // Drive Right to push duck
  setAllMotorSpeeds(0, -0.1, 0);
  PidDelay(2700);

  // back up and away from duck
  setAllMotorSpeeds(0, 0.1, 0);
  PidDelay(500);
  setAllMotorSpeeds(-0.1, 0, 0);
  PidDelay(1000);

  //rotate
  setAllMotorSpeeds(0, 0, -0.25);
  PidDelay(2000);

  // Move to crater
  setAllMotorSpeeds(0.1, 0, 0);
  PidDelay(2500);

  //hit far wall 
  setAllMotorSpeeds(0, 0.1, 0);
  PidDelay(2900);

  //go towards knob along wall
  setAllMotorSpeeds(0.1, 0.005, 0);
  PidDelay(5800);

  //back up a bit
  setAllMotorSpeeds(-0.1, 0, 0);
  PidDelay(400);
  setAllMotorSpeeds(0, -0.1, 0);
  PidDelay(800);

  //rotate -90 degrees
  setAllMotorSpeeds(0, 0, -0.25);
  PidDelay(1800);

  //move to far wall, right then forward
  setAllMotorSpeeds(0.05, 0.05, 0);
  PidDelay(1500);
  setAllMotorSpeeds(0, 0.1, 0);
  PidDelay(1200);

  //run to knob
  setAllMotorSpeeds(-0.05, 0.005, 0);
  PidDelay(2000);
  setAllMotorSpeeds(-0.01, 0.01, 0);
  PidDelay(5000);

  //back off knob
  setAllMotorSpeeds(0.05, -0.05, 0);
  PidDelay(1000);

  //rotate little less than 180 to read
  setAllMotorSpeeds(0, 0, 0.25);
  PidDelay(3200);

  // move arm into reading pos & get closer
  SerialAtomics::send(Message::MoveArm);
  SerialAtomics::send(ArmPositions::READ_COLOR);
  setAllMotorSpeeds(0.05, 0, 0);
  PidDelay(1300);

  // align w/ antenna
  setAllMotorSpeeds(0, -0.05, 0);
  PidDelay(1500);

  //pause for read
  SerialAtomics::send(Message::ReadThenSetLED);
  SerialAtomics::send(1);
  setAllMotorSpeeds(0, 0, 0);
  PidDelay(5000);

  //back off antenna
  setAllMotorSpeeds(-0.1, .05, 0);
  PidDelay(900);


  // === HELLDIVERS ===


  // move towards crater
  setAllMotorSpeeds(0, 0.1, 0);
  PidDelay(800);

  // slam into wall behind spinny thing antenna
  setAllMotorSpeeds(0.15, 0.05, 0);
  PidDelay(2000);

  // move left to where helldivers should be deployed
  setAllMotorSpeeds(0.02, 0.1, 0);
  PidDelay(600);

  // DEPLOY THE HELLDIVERS
  SerialAtomics::send(Message::MoveBugs);
  SerialAtomics::send(BugPositions::HELLDIVE);
  PidDelay(3000);
  SerialAtomics::send(Message::MoveBugs);
  SerialAtomics::send(BugPositions::COLLAPSED);
  PidDelay(1000);
  SerialAtomics::send(Message::MoveArm);
  SerialAtomics::send(ArmPositions::COLLAPSED);
  PidDelay(2700);

  // back off wall after deploying HELLDIVERS
  setAllMotorSpeeds(-0.15, 0, 0);
  PidDelay(1500);

  //rotate 90 degrees
  setAllMotorSpeeds(0, 0, 0.25);
  PidDelay(1800);

  // go to start corner
  setAllMotorSpeeds(0.1, 0.01, 0);
  PidDelay(2800);
  setAllMotorSpeeds(0.01, 0.1, 0);
  PidDelay(2800);

  setAllMotorSpeeds(0.05, 0.05, 0);
  PidDelay(300);

  // go towards keypad antenna
  setAllMotorSpeeds(-0.1, 0, 0);
  PidDelay(3000);

  // go towards keypad antenna (slower)
  setAllMotorSpeeds(-0.05, 0, 0);
  PidDelay(1700);

  // back off a bit
  setAllMotorSpeeds(0.05, 0, 0);
  PidDelay(500);

  // align against wall
  setAllMotorSpeeds(0, 0.1, 0);
  PidDelay(300);

  // push keypad antenna duck into blue square
  setAllMotorSpeeds(0, -0.1, 0);
  PidDelay(2200);

  // diagonal to blue square
  setAllMotorSpeeds(-0.1, -0.1, 0);
  PidDelay(800);

  // reverse diagonal to get out of blue square
  setAllMotorSpeeds(0.1, 0.1, 0);
  PidDelay(600);

  // go towards keypad antenna
  setAllMotorSpeeds(0, 0.1, 0);
  PidDelay(500);

  //rotate 180 degrees
  setAllMotorSpeeds(0, 0, 0.25);
  PidDelay(3400);

  // align x to keypad
  setAllMotorSpeeds(0.1, 0, 0);
  PidDelay(1400);

  // go to keypad
  setAllMotorSpeeds(0, -0.1, 0);
  PidDelay(700);

  // diagonal to keypad
  setAllMotorSpeeds(0.035, -0.02, 0);
  PidDelay(1400);

  // drive into keypad to ensure good contact
  setAllMotorSpeeds(0.01, -0.03, 0);
  PidDelay(600);

  // -- TODO --: Knock off middle duck and read antenna then read keypad antenna

  // -- STOP ALL MOTORS --
  setAllMotorSpeeds(0, 0, 0);
  while (true) {
    
  }
}
