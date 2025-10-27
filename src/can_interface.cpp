#include "can_interface.h"
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can; // Use CAN3 to match working code

bool CANInterface::begin(const String& node_role) {
  can.begin();
  can.setBaudRate(1000000); // Match working code: 100 kbps

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

  // Test CAN initialization
  CAN_message_t test_msg;
  test_msg.id = 0x7FF;
  test_msg.len = 0;
  if (!can.write(test_msg)) {
    Serial.println("ERROR: CAN initialization test failed (write)");
    CAN_error_t err;
    if (can.error(err, true)) {
      Serial.print("Initialization error, ESR1: 0x");
      Serial.println(err.ESR1, HEX);
    }
    return false;
  }

  Serial.print("CAN Configured RX ID: 0x");
  Serial.print(rxId, HEX);
  Serial.print(" TX ID: 0x");
  Serial.println(txId, HEX);
  return true;
}

bool CANInterface::readJointCommand(CANJointCommand& cmd) {
  CAN_message_t msg;
  bool got = false;

  while (can.read(msg)) {
    got = true;
    Serial.print("CAN rx id=0x");
    Serial.print(msg.id, HEX);
    Serial.print(" len=");
    Serial.println(msg.len);

    if ((size_t)msg.len < sizeof(float)) {
      Serial.println("ERROR: CAN message too short for velocity");
      continue;
    }

    float velocity = 0.0f;
    memcpy(&velocity, msg.buf, sizeof(float));
    char name[5] = {0};
    size_t name_len = (size_t)msg.len - sizeof(float);
    if (name_len > 4) name_len = 4;
    if (name_len > 0) {
      memcpy(name, msg.buf + sizeof(float), name_len);
    }

    // Debug raw name bytes
    Serial.print("Raw name bytes: ");
    for (size_t i = 0; i < name_len; i++) {
      Serial.print((int)name[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    cmd.joint_name = String(name).trim(); // Trim whitespace or nulls
    cmd.velocity = velocity;
    Serial.print("Parsed velocity=");
    Serial.println(velocity);
    Serial.print("Parsed name=");
    Serial.println(cmd.joint_name);
    return true;
  }

  if (!got) {
    CAN_error_t err;
    if (can.error(err, true)) {
      Serial.print("CAN read error, ESR1: 0x");
      Serial.println(err.ESR1, HEX);
      if (err.ESR1 & 0x2000) Serial.println(" - ACK Error detected");
      if (err.ESR1 & 0x1000) Serial.println(" - Bit0 Error detected");
      if (err.ESR1 & 0x0800) Serial.println(" - Bit1 Error detected");
      if (err.ESR1 & 0x0400) Serial.println(" - CRC Error detected");
      if (err.ESR1 & 0x0020) Serial.println(" - Bus Off state");
    }
    return false;
  }
  return false;
}

bool CANInterface::sendHeartbeat() {
  CAN_message_t msg;
  msg.id = 0x700;
  msg.len = 0;
  bool success = can.write(msg) == 1;
  Serial.print("sendHeartbeat result: ");
  Serial.println(success ? "success" : "failed");
  if (!success) {
    CAN_error_t err;
    if (can.error(err, true)) {
      Serial.print("Heartbeat error, ESR1: 0x");
      Serial.println(err.ESR1, HEX);
      if (err.ESR1 & 0x2000) Serial.println(" - ACK Error detected");
      if (err.ESR1 & 0x1000) Serial.println(" - Bit0 Error detected");
      if (err.ESR1 & 0x0800) Serial.println(" - Bit1 Error detected");
      if (err.ESR1 & 0x0400) Serial.println(" - CRC Error detected");
      if (err.ESR1 & 0x0020) Serial.println(" - Bus Off state");
    }
  }
  return success;
}

void CANInterface::sendJointFeedback(const String& joint_name, float velocity) {
  CAN_message_t msg;
  msg.id = txId;
  msg.len = 8;
  memcpy(msg.buf, &velocity, sizeof(float));
  char name[4] = {'?', '?', '?', '?'};
  joint_name.substring(0, 4).toCharArray(name, 5);
  memcpy(msg.buf + 4, name, 4);

  int retries = 3;
  bool success = false;
  while (retries > 0 && !success) {
    success = can.write(msg) == 1;
    if (!success) {
      retries--;
      delay(10);
    }
  }

  if (!success) {
    Serial.print("ERROR: Failed to send joint feedback for ");
    Serial.print(joint_name);
    Serial.print(" after 3 retries. CAN error state: ");
    CAN_error_t err;
    if (can.error(err, true)) {
      Serial.print("ESR1: 0x");
      Serial.println(err.ESR1, HEX);
      if (err.ESR1 & 0x2000) Serial.println(" - ACK Error detected");
      if (err.ESR1 & 0x1000) Serial.println(" - Bit0 Error detected");
      if (err.ESR1 & 0x0800) Serial.println(" - Bit1 Error detected");
      if (err.ESR1 & 0x0400) Serial.println(" - CRC Error detected");
      if (err.ESR1 & 0x0020) Serial.println(" - Bus Off state");
    } else {
      Serial.println("No error code available");
    }
  } else {
    Serial.print("Sent joint feedback for ");
    Serial.println(joint_name);
  }
}