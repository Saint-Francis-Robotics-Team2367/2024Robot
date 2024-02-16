#pragma once

#include "geometry/Rotation2d.h"
#include "NavX.h"

class FusedGyro
{

private:
    NavX mGyro = NavX();
    Rotation2d trueAngle;

    // drift increment when outside driftBoundDegrees
    double maxDriftDegrees = 10;
    // Range for sensors to be in to proportion drift
    double driftBoundDegrees = 10;
    // Degrees
    double previousRawGyro;

public:
    void init()
    {
        mGyro.init();
        trueAngle = mGyro.getBoundedAngleCW();
        previousRawGyro = trueAngle.getDegrees();
    }

    /**
     * Input in Clockwise positive
     * Run in robot periodic
     * Input should me limelight or magnetometer
     */
    void updatePeriodic(Rotation2d trueDetectedAngle)
    {
        // Convert all angles to 0-360 deg, CW positive, compass system
        double parentAngle = Rotation2d::degreesBound(trueDetectedAngle.getDegrees());
        double currTrueAngle = Rotation2d::degreesBound(trueAngle.getDegrees());
        double currGyroChange = mGyro.getBoundedAngleCW().getDegrees() - previousRawGyro;

        // Determine drift direction
        bool driftPositive = false;
        double positiveDistance = Rotation2d::degreesBound(parentAngle - currTrueAngle);
        double negativeDistance = Rotation2d::degreesBound(currTrueAngle - parentAngle);
        driftPositive = positiveDistance < negativeDistance;

        double angleError = driftPositive ? positiveDistance : negativeDistance;
        double driftIncrement;

        // Determine bounds
        if (fabs(angleError) < driftBoundDegrees)
        {
            driftIncrement = ((driftBoundDegrees - angleError) / driftBoundDegrees) * maxDriftDegrees;
        }
        else
        {
            driftIncrement = maxDriftDegrees;
        }
        driftIncrement = driftPositive ? driftIncrement : -driftIncrement;

        // Add drift and sensor changes
        currTrueAngle += (driftIncrement + currGyroChange);
        trueAngle = Rotation2d::fromDegrees(Rotation2d::degreesBound(currTrueAngle));
    }

    // Get angle with clockwise positive
    Rotation2d getAngleCW()
    {
        return trueAngle;
    }

    // Get angle with counterclockwise positive
    Rotation2d getAngleCCW()
    {
        return Rotation2d::fromDegrees(Rotation2d::degreesBound(-trueAngle.getDegrees()));
    }
};