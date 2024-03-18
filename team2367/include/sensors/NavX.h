#pragma once

#include "AHRS.h"
#include "geometry/Rotation2d.h"
#include "frc/geometry/Rotation2d.h"

class NavX
{
public:
    AHRS gyro = AHRS(frc::SerialPort::kMXP);
    Rotation2d angleOffset = Rotation2d(0.0);
    
    void init()
    {
        gyro.Reset();
        angleOffset = Rotation2d(0.0);
    }

    void setOffset(Rotation2d angleOffsetInput) 
    {
        angleOffset = angleOffsetInput;
    }

    Rotation2d getMagnetometerCW()
    {
        return Rotation2d::fromDegrees(gyro.GetCompassHeading());
    }

    /**
     * Radians
     * Counter clockwise(idk why this is stupid)
     */
    Rotation2d getBoundedAngleCCW()
    {
        return Rotation2d(Rotation2d::degreesBound(-gyro.GetAngle() - angleOffset.getDegrees()) * PI / 180);
    }

    Rotation2d getBoundedAngleCW()
    {
        return Rotation2d(Rotation2d::degreesBound(gyro.GetAngle() + angleOffset.getDegrees()) * PI / 180);
    }

    frc::Rotation2d getRotation2d()
    {
        return frc::Rotation2d(units::radian_t((gyro.GetAngle() + angleOffset.getDegrees()) * PI / 180));
    }
};