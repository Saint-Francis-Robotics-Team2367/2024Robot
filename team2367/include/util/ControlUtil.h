#pragma once

#include <cmath>
#include "Constants.h"

class ControlUtil 
{
    public:
    
    /**
     * Add deadzone and scale from deadzone by square
     * Input must be in [-1, 1]
    */
    static double deadZoneQuadratic(double input, double deadzone)
    {
        if (fabs(input) < deadzone) {
            return 0.0;
        } else {
            // y = a(x-dz)^2
            // a = 1 / (1 - dz)^2
            return std::copysign((1 / pow(1 - deadzone, 2)) * pow((fabs(input) - deadzone), 2), input);
        }

    }

    /**
     * Add deadzone and scale from deadzone by square
     * Input must be in [-1, 1]
    */
    static double deadZonePower(double input, double deadzone, int power)
    {
        if (fabs(input) < deadzone) {
            return 0.0;
        } else {
            // y = a(x-dz)^2
            // a = 1 / (1 - dz)^2
            return std::copysign((1 / pow(1 - deadzone, power)) * pow((fabs(input) - deadzone), power), input);
        }

    }

    /**
     * Takes double, returns 0.0 if
     * if absolute val less than 1e-12
     * else returns input
     * kEpsilon=1e-12 defined in Constants
    */
    static double epsilonBound(double a) 
    {
        if (fabs(a) < kEpsilon) 
        {
            return 0.0;
        } else {
            return a;
        }

    }





    



};