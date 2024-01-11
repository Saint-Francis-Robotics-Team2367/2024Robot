#pragma once

#include "AHRS.h"
#include "geometry/Rotation2d.h"


class NavX {
    public:


    AHRS gyro = AHRS(frc::SerialPort::kMXP);


    void init() {
        gyro.Reset();

    }


    /**
     * Radians
     * Counter clockwise(idk why this is stupid)
    */
    Rotation2d getBoundedAngleCCW() {
        return Rotation2d(Rotation2d::degreesBound(-gyro.GetAngle()) * M_PI / 180);
    }

    Rotation2d getBoundedAngleCW() {
        return Rotation2d(Rotation2d::degreesBound(gyro.GetAngle()) * M_PI / 180);
    }

    




};