#pragma once

#include "geometry/Translation2d.h"
#include "geometry/Rotation2d.h"
#include "geometry/Twist2d.h"
#include <cmath>
#include "util/ControlUtil.h"

class Pose2d
{
public:
    Pose2d() : translation_(Translation2d()), rotation_(Rotation2d()) {}

    Pose2d(double x, double y, const Rotation2d &rotation)
        : translation_(Translation2d(x, y)), rotation_(rotation) {}
    Pose2d(const Translation2d &translation, const Rotation2d &rotation)
        : translation_(translation), rotation_(rotation) {}

    Translation2d getTranslation() const { return translation_; }
    Rotation2d getRotation() const { return rotation_; }

    static Twist2d log(Pose2d transform)
    {
        double dtheta = transform.getRotation().getRadians();
        double half_dtheta = 0.5 * dtheta;
        double cos_minus_one = cos(transform.getRotation().getRadians()) - 1.0;
        double halftheta_by_tan_of_halfdtheta;
        if (fabs(cos_minus_one) < kEpsilon)
        {
            halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        }
        else
        {
            halftheta_by_tan_of_halfdtheta = -(half_dtheta * sin(transform.getRotation().getRadians())) / cos_minus_one;
        }
        Translation2d translation_part = transform.getTranslation()
                                             .rotateBy(Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
        return Twist2d(translation_part.x(), translation_part.y(), dtheta);
    }

private:
    Translation2d translation_;
    Rotation2d rotation_;
};
