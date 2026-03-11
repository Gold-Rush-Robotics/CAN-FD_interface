#include <ColorSensor.h>
#include <LED.h>
#include <Arm.h>
#include <timer.h>


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
    Timer timer = Timer();
    // testAllLeds();


    //Set start pose
    Servos::move(3, BugPositions::COLLAPSED);
    Servos::moveArm(ArmPositions::READ_COLOR);
    delay(1000);
    Servos::moveArm(ArmPositions::COLLAPSED);

    timer.waitUntil(6400);
    Servos::moveArm(ArmPositions::READ_COLOR);

    timer.waitUntil(8300);
    ColorSensor::readThenSetLED(0);  

    //back up
    timer.waitUntil(10400);
    Servos::moveArm(ArmPositions::COLLAPSED);

    //at knob antenna
    timer.waitUntil(44500);
    Servos::moveArm(ArmPositions::READ_COLOR);
    timer.waitUntil(45500);
    ColorSensor::readThenSetLED(1);
    timer.waitUntil(47500);
    Servos::moveArm(ArmPositions::COLLAPSED);


    //stop
    delay(10000000);
}
