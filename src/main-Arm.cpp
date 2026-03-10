#include <ColorSensor.h>
#include <LED.h>
#include <Arm.h>


#define ServoSerialPort Serial1


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
    ServoSerialPort.begin(1000000);
    Servos::controller.pSerial = &ServoSerialPort;
}

void loop() {
    // testAllLeds();
    // Servos::moveArm(ArmPositions::READ_COLOR);
    // delay(100000);

    //Set start pose
    Servos::moveArm(ArmPositions::COLLAPSED);
    delay(6200);

    Servos::moveArm(ArmPositions::READ_COLOR);
    delay(1000);
    delay(900);
    delay(1000);
    ColorSensor::readThenSetLED(0);  
    delay(1000);    

    //back up
    delay(600);
    Servos::moveArm(ArmPositions::COLLAPSED);


    //stop
    delay(10000000);
}
