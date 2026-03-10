#ifndef CAN_INTERFACE_H
#define CAN_INTERFACE_H

#include <Arduino.h>

struct CANJointCommand {
  String joint_name;
  float velocity;
};

class CANInterface {
public:
  bool begin(const String& node_role);
  bool readJointCommand(CANJointCommand& cmd);
  bool sendHeartbeat();
  void sendJointFeedback(const String& joint_name, float velocity);

private:
  uint32_t rxId, txId;
  // Assume 'can' is a FlexCAN_T4 or similar CAN object, defined externally
};

#endif