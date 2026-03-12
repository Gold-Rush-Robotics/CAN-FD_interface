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
    delay(300);
    Servos::moveArm(ArmPositions::COLLAPSED);


    #ifndef POS_MODE
    // Wait for mecanum board to setup
    Message msg = Message::Invalid;
    while (msg != Message::Ping) {
        msg = SerialAtomics::recvMsg();
    }
    SerialAtomics::send(Message::Pong);

    // Wait for start LED, then tell mecanum teensy to start the game
    while (true) {
        uint16_t lux = ColorSensor::readLux();

        if (lux >= 0) {
            SerialAtomics::send(Message::StartGame);
            break;
        }
    }
    #endif
}

void loop() {
    #ifdef POS_MODE
    Serial.setTimeout(100000);
    while (true) {
        Serial.print("Servo 1: ");
        Servos::move(1, Serial.readStringUntil('\n', 3).toInt());
        Serial.print("Servo 2: ");
        Servos::move(2, Serial.readStringUntil('\n', 3).toInt());
    }
    #endif

    while (true) {
        switch (SerialAtomics::recvMsg()) {
            case Message::MoveArm:
                Servos::moveArm(SerialAtomics::recvByte());
                break;
            case Message::MoveBugs:
                Servos::move(3, SerialAtomics::recvByte());
                break;
            case Message::ReadThenSetLED:
                ColorSensor::readThenSetLED((size_t) SerialAtomics::recvByte());
                break;
            default:
                Serial.println("Error: Received unexpected message in gameplay loop");
        }
    }
}
