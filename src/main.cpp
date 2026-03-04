// Include Arduino core library
#include <Arduino.h>
// Color Sensor Includes
#include <math.h>
#include <vector>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
// Servo Includes
#include <SCServo.h>


#define ServoSerialPort Serial1

SMS_STS sms_sts;

struct Color {
  float r;
  float g;
  float b;
};

const Color red = {234.0, 13.0, 24.0};
const Color green = {23.0, 150.0, 77};
const Color blue = {10.0, 50.0, 200.0}; 
const Color purple = {110.0, 35.0, 119.0};

enum ColorMatch {
    RED = 1,
    GREEN = 2,
    BLUE = 3,
    PURPLE = 4
};
const Color colorMatches[] = {red, green, blue, purple};

void debug(Color color, float redDistance, float greenDistance, float blueDistance, float purpleDistance) {
  // Serial Prints
  Serial.print("\nR: "); Serial.print(color.r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(color.g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(color.b, DEC); Serial.println(" ");

  Serial.print("Distance from red: ");
  Serial.println(redDistance);

  Serial.print("Distance from green: ");
  Serial.println(greenDistance);

  Serial.print("Distance from blue: ");
  Serial.println(blueDistance);

  Serial.print("Distance from purple: ");
  Serial.println(purpleDistance);
}

class ColorSensor {
    private:
    float r, g, b;
    Adafruit_TCS34725 tcs;

    public:
    // LED color vectors
    ColorSensor() {
        tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

        if (!tcs.begin()) {
            Serial.println("Color sensor failed to connect");
        }
    }

    // Function to calculate the "distance" between the current color and a target color using Euclidean distance in RGB space.
    float calculateEvilDistance(Color current, Color match) {
        // k-nearest neighbors >:)
        float diffR, diffG, diffB;
        diffR = current.r - match.r;
        diffG = current.g - match.g;
        diffB = current.b - match.b;
        
        long sumOfSquares = (long)diffR * diffR + (long)diffG * diffG + (long)diffB * diffB;

        
        return sqrt(sumOfSquares);
    }

    ColorMatch calculateMatch() {
        tcs.getRGB(&r, &g, &b);
        Color read = {r, g, b};

        float redDistance = calculateEvilDistance(read, red);
        float greenDistance = calculateEvilDistance(read, green);
        float blueDistance = calculateEvilDistance(read, blue);
        float purpleDistance = calculateEvilDistance(read, purple);

        debug(read, redDistance, greenDistance, blueDistance, purpleDistance);

        float minDistance = min(min(redDistance, greenDistance), min(blueDistance, purpleDistance));

        if (minDistance == redDistance) {
            return ColorMatch::RED;
        } else if (minDistance == greenDistance) {
            return ColorMatch::GREEN;
        } else if (minDistance == blueDistance) {
            return ColorMatch::BLUE;
        } else {
            return ColorMatch::PURPLE;
        }
    }
};
ColorSensor sensor;

// LED pin definitions. Each LED (4 in total) has 3 pins for R, G, and B respectively.
int leds[4][3] = {
    {13, 14, 15},
    {41, 40, 39},
    {38, 37, 36},
    {35, 34, 33}
};

// Function to set the position of a servo with a given ID. Position is mapped from 0-360 degrees to 0-4096 for the servo controller.
// Servos are 1 and 2.
void setServoPosition(int pos, int servo_ID) {
    int piss = map(pos, 0, 360, 0, 4096);
    sms_sts.WritePosEx(servo_ID, piss, 2400, 50);
}

// Function to set the color of a specific LED (0-3) using a Color struct.
void setColor(int led, bool r, bool g, bool b) {
    digitalWrite(leds[led][0], r);
    digitalWrite(leds[led][1], g);
    digitalWrite(leds[led][2], b);  
}

void setup() {
    // Initialize LED pins as OUTPUT
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            pinMode(leds[i][j], OUTPUT);
        }
    }

    // Start sensor
    sensor = ColorSensor();

    // Setup for Servo
    // ServoSerialPort.begin(1000000);
    sms_sts.pSerial = &ServoSerialPort;
    delay(1000);
}

void loop() {
    ColorMatch ret = sensor.calculateMatch();
    if (ret == ColorMatch::RED) {
        setColor(0, HIGH, LOW, LOW);
    } else if (ret == ColorMatch::GREEN) {
        setColor(0, LOW, HIGH, LOW);
    } else if (ret == ColorMatch::BLUE) {
        setColor(0, LOW, LOW, HIGH);
    } else if (ret == ColorMatch::PURPLE) {
        setColor(0, HIGH, LOW, HIGH);
    }

    delay(2000);
    setColor(0, LOW, LOW, LOW);
    delay(2000);
}