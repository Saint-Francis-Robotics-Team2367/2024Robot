#pragma once
#include "geometry/Rotation2d.h"

class SwerveModuleState {
    private:
        double speed_;
        double angleRadians_;

    public:
        SwerveModuleState(double speedFPS, double angleRadians) {
            speed_ = speedFPS;
            angleRadians_ = angleRadians;
        }

        SwerveModuleState(double speedFPS, Rotation2d rotRadians) {
            speed_ = speedFPS;
            angleRadians_ = rotRadians.getRadians();
        }
        SwerveModuleState() {
            speed_ = 0;
            angleRadians_ = 0;
        }
        Rotation2d getRot2d() {
            return Rotation2d(angleRadians_);
        }
        double getSpeedFPS() {
            return speed_;
        }
        void setSpeedFPS(const double speedFPS) {
            speed_ = speedFPS;
        }
        void setRot2d(const Rotation2d angle) {
            angleRadians_ = angle.getRadians();
        }





};