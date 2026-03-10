// #include <SCServo.h>

// // Disclaimer: I have no clue what the below says, I just want to make the servo move
// // 1. 连接舵机到 Serial1
// // 2. 将 Serial1 的波特率设置为 1000000
// // 3. 运行程序，舵机会在 0° 和 90° 之间来回摆动，每个位置停留 2 秒钟
// #define ServoSerialPort Serial1

// SMS_STS sms_sts;

// void setServoPosition(int pos, int servo_ID) {
//   int piss = map(pos, 0, 360, 0, 4096);
//   sms_sts.WritePosEx(servo_ID, piss, 2400, 50);
// }

// void setup() {
//   ServoSerialPort.begin(1000000);
//   sms_sts.pSerial = &ServoSerialPort;
//   delay(1000);
// }

// void loop() {
//   setServoPosition(0, 1);
//   delay(2000);
//   setServoPosition(90, 1);
//   delay(2000);
//   setServoPosition(0, 2);
//   delay(2000);
//   setServoPosition(270, 2);
//   delay(2000);
// }