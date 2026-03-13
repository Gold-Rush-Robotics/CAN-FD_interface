#define ServoSerialPort Serial1

// Uncomment to make arm just print color readings and not do game loop
// #define COLOR_SENSOR_TUNING

#ifdef COLOR_SENSOR_TUNING
#define START_LUX 9999
#else
#define START_LUX 50
#endif

#include <ColorSensor.h>
#include <LED.h>
#include <Arm.h>
#include <timer.h>
#include <SerialAtomics.h>


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

    // Wait for mecanum board to setup
    Message msg = Message::Invalid;
    while (msg != Message::Ping) {
        msg = SerialAtomics::recvMsg();
    }
    SerialAtomics::send(Message::Pong);

    // Wait for start LED, then tell mecanum teensy to start the game
    while (true) {
        uint16_t lux = ColorSensor::readLux();

        // for(int i = 0; i < 100; i++) {
        //     lux = ColorSensor::readLux();
        //     Serial.println(lux);
        //     delay(10);
        // }
        if (lux <= START_LUX) {
            SerialAtomics::send(Message::StartGame);
            break;
        }
    }
}

void loop() {
    #ifdef COLOR_SENSOR_TUNING
    Servos::moveArm(ArmPositions::READ_COLOR);
    while (true) {
        setLedColor(0, LOW, LOW, LOW);
        ColorSensor::readThenSetLED(0);
        delay(3000);
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
