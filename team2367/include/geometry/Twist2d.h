#pragma once

#include <cmath>

class Twist2d {
private:
    double dx_;
    double dy_;
    double dtheta_;


public:
    Twist2d(double dx, double dy, double dtheta) {
        dx_ = dx;
        dy_ = dy;
        dtheta_ = dtheta;
    }

    double norm() {
        return sqrt((dx_ * dx_) + (dy_ * dy_));
    }

    Twist2d scale(double scale_val) {
        return Twist2d(dx_ * scale_val, dy_ * scale_val, dtheta_ * scale_val);
    }

    Twist2d mirror() {
        return Twist2d(dx_, -dy_, -dtheta_);
    }

    



};