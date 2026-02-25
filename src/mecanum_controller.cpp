#include "mecanum_controller.h"

MecanumController::MecanumController(float wheelBase, float trackWidth, float wheelDiameter)
    : wheelBase(wheelBase), trackWidth(trackWidth), wheelDiameter(wheelDiameter) {
}

float MecanumController::getWheelBase() {
    return wheelBase;
}

float MecanumController::getWheelDiameter() {
    return wheelDiameter;
}

float MecanumController::getTrackWidth() {
    return trackWidth;
}

float* MecanumController::calculateMecanumWheelSpeeds(float linear_x, float linear_y, float angular_z) {
    static float wheelSpeeds[4];
    
    // Calculate the kinematics factor
    float k = (1.0f / wheelDiameter);
    float offset = (wheelBase + trackWidth);

    wheelSpeeds[0] = k * (linear_x - linear_y - offset * angular_z);
    wheelSpeeds[1] = k * (linear_x + linear_y + offset * angular_z);
    wheelSpeeds[2] = k * (linear_x + linear_y - offset * angular_z);
    wheelSpeeds[3] = k * (linear_x - linear_y + offset * angular_z);
    
    return wheelSpeeds;
}

