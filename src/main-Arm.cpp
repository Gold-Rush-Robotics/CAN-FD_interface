#include <ColorSensor.h>
#include <LED.h>
#include <Arm.h>
#include <timer.h>
#include <SerialAtomics.h>


#define ServoSerialPort Serial1


void setup() {
    // USB serial for printing
    Serial.begin(115200);

    // Serial comm. with mecanum teensy
    SerialAtomics::setup();

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

    // Move to start pos
    Servos::move(3, BugPositions::COLLAPSED);
    Servos::moveArm(ArmPositions::COLLAPSED);


    // Wait for mecanum board to setup
    Message msg = SerialAtomics::recvMsg();
    if (msg != Message::Ping) {
        Serial.println("ERROR: Didn't recieve `ping` from mecanum teensy");
    }
    SerialAtomics::send(Message::Pong);

    // Wait for start LED, then tell mecanum teensy to start the game
    while (true) {
        uint16_t lux = ColorSensor::readLux();

        if (lux >= 500) {
            SerialAtomics::send(Message::StartGame);
            break;
        }
    }
}

void loop() {
    while (true) {
        switch (SerialAtomics::recvMsg()) {
            case Message::MoveArm:
                Servos::moveArm(SerialAtomics::recvByte());
            case Message::MoveBugs:
                Servos::move(3, SerialAtomics::recvByte());
            case Message::ReadThenSetLED:
                ColorSensor::readThenSetLED((size_t) SerialAtomics::recvByte());
            default:
                Serial.println("Error: Received unexpected message in gameplay loop");
        }
    }
}
