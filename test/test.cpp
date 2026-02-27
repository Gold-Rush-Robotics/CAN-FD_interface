#include <SCServo.h>

SCSCL sc;

void setup()
{
  Serial.begin(115200);        // USB debug (separate from Serial1)

  Serial1.begin(1000000);      // Uses (by default): Pin 0 = RX1, Pin 1 = TX1

  sc.pSerial = &Serial1;       // Attach FTServo to Serial1
}

void loop()
{
  sc.WritePos(1, 1000, 0, 1500);
  delay(1000);

  sc.WritePos(1, 20, 0, 1500);
  delay(1000);
}