#include "can_interface.h"

void CANInterface::begin(const String& node_role) {
  CANFD_timings_t config;
  config.clock = CLK_20MHz;
  config.baudrate = 1000000;
//   config.baudrateFD = 2000000;
  config.propdelay = 190;
  config.bus_length = 1;
  config.sample = 70;

//   canfd.begin();
//   canfd.setRegions(64);
//   canfd.setBaudRate(config);
    can.begin();
    // can.setRegions(64);
    can.setBaudRate(1000000);

  // Assign CAN IDs based on role
  if (node_role == "FRONT") {
    rxId = 0x100;
    txId = 0x101;
  } else if (node_role == "REAR") {
    rxId = 0x200;
    txId = 0x201;
  } else if (node_role == "LEFT") {
    rxId = 0x300;
    txId = 0x301;   
  } else if (node_role == "RIGHT") {
    rxId = 0x400;
    txId = 0x401;
  } else {
    rxId = 0x7FF;
    txId = 0x7FF;
  }

  Serial.print("CAN Configured RX ID: 0x");
  Serial.print(rxId, HEX);
  Serial.print(" TX ID: 0x");
  Serial.println(txId, HEX);
}

bool CANInterface::readJointCommand(CANJointCommand& cmd) {
//   CANFD_message_t msg;
    CAN_message_t msg;
//   if (canfd.read(msg)) {
    if (can.read(msg)) {
        float velocity;
        memcpy(&velocity, msg.buf, sizeof(float));

        char name[5] = {0};
        memcpy(name, msg.buf + 4, 4);

        cmd.joint_name = String(name);
        cmd.velocity = velocity;
        return true;
    }
  return false;
}
bool CANInterface::sendHeartbeat() {
    CAN_message_t msg;
    msg.id = 0x700; // Heartbeat ID
    msg.len = 0;    // No data

    return can.write(msg) == 1;
}

void CANInterface::sendJointFeedback(const String& joint_name, float velocity) {
  //   CANFD_message_t msg;
    CAN_message_t msg;
    msg.id = txId;
    msg.len = 8;

    memcpy(msg.buf, &velocity, sizeof(float));

    char name[4] = {'?', '?', '?', '?'};
    joint_name.substring(0, 4).toCharArray(name, 5);
    memcpy(msg.buf + 4, name, 4);

    can.write(msg);
}
