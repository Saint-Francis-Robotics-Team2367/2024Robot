#pragma once

#include "geometry/Translation2d.h"
#include "geometry/Rotation2d.h"

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


private:
    Translation2d translation_;
    Rotation2d rotation_;
};
