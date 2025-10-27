#pragma once
#include <Arduino.h>
#include <FlexCAN_T4.h>

struct CANJointCommand {
    String joint_name;
    float velocity;
};

class CANInterface {
public:
    void begin(const String& node_role);
    bool readJointCommand(CANJointCommand& cmd);
    void sendJointFeedback(const String& joint_name, float velocity);
    bool sendHeartbeat();

private:
//   FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> canfd;
    FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can;
    uint32_t txId;
    uint32_t rxId;
};
