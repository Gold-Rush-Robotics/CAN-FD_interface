// Pin definitions for Motor 1
# include <Encoder.h>
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

// Target RPM for motors
const int targetRPM = 50;
Encoder enc2(EN_OUTB1, EN_OUTA1);
Encoder enc1(EN_OUTB2, EN_OUTA2);
const int PWM_FREQ = 20000;     // Hz
const int ticksPerRev = 64;    // encoder resolution                                                                                                                                                                                                                
const float gearRatio = 30.0; 

// Function to initialize motor pins
void setupMotorPins() {
  // Motor 1 pins
  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(SLP1, OUTPUT);
  pinMode(FLT1, INPUT);
  // pinMode(EN_OUTA1, INPUT);
  // pinMode(EN_OUTB1, INPUT);
  pinMode(CS1, INPUT);

  // Motor 2 pins
  pinMode(DIR2, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(SLP2, OUTPUT);
  pinMode(FLT2, INPUT);
  // pinMode(EN_OUTA2, INPUT);
  // pinMode(EN_OUTB2, INPUT);
  pinMode(CS2, INPUT);

}

// Function to set motor speed and direction
void setMotorSpeed(int dirPin, int pwmPin, int slpPin, int speed) {
  digitalWrite(slpPin, HIGH); // Wake up the motor driver
  if (speed >= 0) {
    digitalWrite(dirPin, HIGH); // Set direction forward
  } else {
    digitalWrite(dirPin, LOW); // Set direction reverse
    speed = -speed; // Make speed positive
  }
  analogWrite(pwmPin, speed); // Set motor speed
}
void print_motor_status() {
  Serial.print("Motor 1 - ENC: "); Serial.print(enc2.read());

  Serial.print(" | Motor 2 - ENC: "); Serial.println(enc1.read());
}

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);

  // Initialize motor pins
  setupMotorPins();

  // Wake up both motors
  digitalWrite(SLP1, HIGH);
  digitalWrite(SLP2, HIGH);

  // delay(60000);
  
  
  
  
  // delay(2000);
  
  // setMotorSpeed(DIR1, PWM1, SLP1, 0);
  // setMotorSpeed(DIR2, PWM2, SLP2, 0);
  // delay(2000);
  // setMotorSpeed(DIR1, PWM1, SLP1, -50);
  // delay(10);
  // setMotorSpeed(DIR2, PWM2, SLP2, 50);
  // delay(5000);
  // setMotorSpeed(DIR1, PWM1, SLP1, 0);
  // setMotorSpeed(DIR2, PWM2, SLP2, 0);

  // Set both motors to 100 RPM
  // Assuming a mapping of RPM to PWM value (e.g., 100 RPM = 128 PWM)
  // setMotorSpeed(DIR1, PWM1, SLP1, map(100, -100, 100, -255, 255));
  // setMotorSpeed(DIR2, PWM2, SLP2, map(-100, -100, 100, -255, 255));
}
//2 left bin grabber 87
void loop() {
  setMotorSpeed(DIR2, PWM2, SLP2, 100);
  setMotorSpeed(DIR1, PWM1, SLP1, -100);
  print_motor_status();

  


  // Set both motors to 100 RPM
  // Assuming a mapping of RPM to PWM value (e.g., 100 RPM = 128 PWM)
  // setMotorSpeed(DIR1, PWM1, SLP1, map(100, -100, 100, -255, 255));
  // setMotorSpeed(DIR2, PWM2, SLP2, map(-100, -100, 100, -255, 255));
}
