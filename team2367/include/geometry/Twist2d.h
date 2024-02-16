#pragma once

#include <cmath>

class Twist2d
{
public:
    double dx;
    double dy;
    double dtheta;

    Twist2d(double dx_, double dy_, double dtheta_)
    {
        dx = dx_;
        dy = dy_;
        dtheta = dtheta_;
    }

    double norm()
    {
        return sqrt((dx * dx) + (dy * dy));
    }

    Twist2d scale(double scale_val)
    {
        return Twist2d(dx * scale_val, dy * scale_val, dtheta * scale_val);
    }

    Twist2d mirror()
    {
        return Twist2d(dx, -dy, -dtheta);
    }
};