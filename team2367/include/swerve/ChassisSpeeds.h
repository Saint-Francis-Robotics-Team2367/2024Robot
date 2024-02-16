#pragma once

#include <cmath>
#include <string>

#include "geometry/Rotation2d.h"

class ChassisSpeeds {
public:
    double vxMetersPerSecond;
    double vyMetersPerSecond;
    double omegaRadiansPerSecond;

    ChassisSpeeds()
        : vxMetersPerSecond(0.0),
          vyMetersPerSecond(0.0),
          omegaRadiansPerSecond(0.0) {}

    ChassisSpeeds(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond)
        : vxMetersPerSecond(vxMetersPerSecond),
          vyMetersPerSecond(vyMetersPerSecond),
          omegaRadiansPerSecond(omegaRadiansPerSecond) {}

    static ChassisSpeeds fromFieldRelativeSpeeds(double vxMetersPerSecond, double vyMetersPerSecond,
                                                 double omegaRadiansPerSecond, const Rotation2d robotAngle) {
        return ChassisSpeeds(
            vxMetersPerSecond * cos(robotAngle.getRadians()) + vyMetersPerSecond * sin(robotAngle.getRadians()),
            -vxMetersPerSecond * sin(robotAngle.getRadians()) + vyMetersPerSecond * cos(robotAngle.getRadians()),
            omegaRadiansPerSecond);
    }

    static ChassisSpeeds fromRobotRelativeSpeeds(double vxMetersPerSecond, double vyMetersPerSecond,
                                                 double omegaRadiansPerSecond) {
        return ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
    }

    // static ChassisSpeeds fromFieldRelativeSpeeds(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond, Rotation2d robotAngle) {
    //     return ChassisSpeeds(
    //             vxMetersPerSecond * robotAngle.cos() + vyMetersPerSecond * robotAngle.sin(),
    //             -vxMetersPerSecond * robotAngle.sin() + vyMetersPerSecond * robotAngle.cos(),
    //             omegaRadiansPerSecond);
    // }


    std::string toString() const {
        return "ChassisSpeeds(Vx: " + std::to_string(vxMetersPerSecond) + " m/s, Vy: " +
               std::to_string(vyMetersPerSecond) + " m/s, Omega: " +
               std::to_string(omegaRadiansPerSecond) + " rad/s)";
    }
};
