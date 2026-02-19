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
    
    // Calculate wheel speeds based on inverse kinematics for mecanum drive based on wheel diameter
    // Front Left, Front Right, Rear Left, Rear Right
    wheelSpeeds[0] = (1 / wheelDiameter) * (linear_x - linear_y - (wheelBase + trackWidth) * angular_z); // Front Left
    wheelSpeeds[1] = (1 / wheelDiameter) * (linear_x + linear_y + (wheelBase + trackWidth) * angular_z); // Front Right
    wheelSpeeds[2] = (1 / wheelDiameter) * (linear_x + linear_y - (wheelBase + trackWidth) * angular_z); // Rear Left
    wheelSpeeds[3] = (1 / wheelDiameter) * (linear_x - linear_y + (wheelBase + trackWidth) * angular_z); // Rear Right
    
    return wheelSpeeds;
}

