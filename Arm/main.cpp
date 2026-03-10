// #define COLOR_SENSOR_DEBUG
// #define ARM_DEBUG

#include "ColorSensor.h"
#include "LED.h"
#include "Arm.h"


void setup() {
    // USB serial for printing
    Serial.begin(115200);

    // LEDs
    for (size_t i = 0; i < std::size(LEDS); i++) {
        for (size_t j = 0; j < 3; j++) {
            pinMode(LEDS[i][j], OUTPUT);
        }
    }

    // Color sensor
    ColorSensor::sensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
    if (!ColorSensor::sensor.begin()) {
        Serial.println("ERROR: Color sensor failed to connect");
    }
    
    // Arm
    #define ServoSerialPort Serial1
    ServoSerialPort.begin(1000000);
    Arm::controller.pSerial = &ServoSerialPort;
    
    // Wait to ensure everything sets up
    delay(1000);
}

void loop() {
    Arm::move(ArmPositions::READ_COLOR);

    delay(3000);
    ColorSensor::readThenSetLED(0);
    
    Arm::move(ArmPositions::COLLAPSED);

    delay(3000);
}