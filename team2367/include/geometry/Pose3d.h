#pragma once
#include <vector>

class Pose3d
{
public:
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;

    Pose3d(std::vector<double> pose)
    {
        this->x = pose[0];
        this->y = pose[1];
        this->z = pose[2];
        this->roll = pose[3];
        this->pitch = pose[4];
        this->yaw = pose[5];
    }
    Pose3d() {
        this->x = 0.0;
        this->y = 0.0;
        this->z = 0.0;
        this->roll = 0.0;
        this->pitch = 0.0;
        this->yaw = 0.0;
    }
};