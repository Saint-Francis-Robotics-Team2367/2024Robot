#pragma once

#include "AHRS.h"
#include "geometry/Rotation2d.h"


class NavX {
    public:


    AHRS gyro = AHRS(frc::SerialPort::kMXP);


    void init() {
        gyro.Reset();

    }

    double getBoundedAngle() {
        return Rotation2d::degreesBound(-gyro.GetAngle()) * M_PI / 180;
    }

    




};