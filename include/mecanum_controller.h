#ifndef MECANUM_CONTROLLER_H
#define MECANUM_CONTROLLER_H

class MecanumController {
public:
    MecanumController(float wheelBase, float trackWidth, float wheelDiameter = 0.075f);
    float getWheelBase();
    float getTrackWidth();
    float getWheelDiameter();

    float* calculateMecanumWheelSpeeds(float linear_x, float linear_y, float angular_z);

private:
    float wheelBase;
    float trackWidth;
    float wheelDiameter; 

};

#endif
