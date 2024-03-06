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
        if (fabs(input) < deadzone)
        {
            return 0.0;
        }
        else
        {
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
        if (fabs(input) < deadzone)
        {
            return 0.0;
        }
        else
        {
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
        }
        else
        {
            return a;
        }
    }

    static bool epsilonEquals(double a, double b)
    {
        if (fabs(a - b) < kEpsilon)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    static double limitAcceleration(double currentVelocity, double desiredVelocity, float maxAcc, float dt)
    {
        float accRange = maxAcc * dt;
        if (fabs(currentVelocity - desiredVelocity) > accRange)
        {
            if (desiredVelocity > currentVelocity)
            {
                desiredVelocity = currentVelocity + accRange;
            }
            else
            {
                desiredVelocity = currentVelocity - accRange;
            }
        }
        return desiredVelocity;
    }

    static float limitPositiveAcceleration(float previousVelocity, float desiredVelocity, float maxAcc, float dt)
    {
        float range = maxAcc * dt;
        if (std::signbit(desiredVelocity) == std::signbit(previousVelocity) && std::abs(desiredVelocity) > std::abs(previousVelocity)) {
            float maxAllowedVelocity = std::abs(previousVelocity) + range;
            float minAllowedVelocity = std::abs(previousVelocity) - range;
            return std::clamp(desiredVelocity, minAllowedVelocity, maxAllowedVelocity);
        }
        return desiredVelocity;

        // if (fabs(desiredVelocity) < kEpsilon) {
        //     return desiredVelocity;
        // }

        // if (previousVelocity > 0 && desiredVelocity > 0)
        // {
        //     if (previousVelocity + range < desiredVelocity)
        //     {
        //         desiredVelocity = previousVelocity + range;
        //     }
        // }
        // else if (previousVelocity < 0 && desiredVelocity < 0)
        // {
        //     if (previousVelocity - range > desiredVelocity)
        //     {
        //         desiredVelocity = previousVelocity - range;
        //     }
        // }
        // else
        // {
        //     if (previousVelocity < 0)
        //     {
        //         desiredVelocity = previousVelocity + range;
        //     }
        //     else
        //     {
        //         desiredVelocity = previousVelocity - range;
        //     }
        // }
        // return desiredVelocity;
    }

    static double min(double a, double b)
    {
        if (a > b)
        {
            return b;
        }
        else
        {
            return a;
        }
    }


    /**
     *  * Increases allowed max velocity to boostPercent when boost boolean is true
     * Otherwise sets allowed maxVelocity to its normal percentage limit
    */
    static double boostScaler(double axis, bool boost, float boostPercent, float normalPercent) 
    {
        if (boost) {
            return axis * boostPercent;
        } 
        return axis * normalPercent;
    }

    static double scaleSwerveVelocity(double desiredVelocity, double angleError, bool quartic) 
    {
        int n = quartic ? 4 : 2; // n is quadratic or quartic
        return (pow(2 / PI, n) * desiredVelocity) * pow(angleError - PI_2, n);


    }
    
    /**
     * returns true if timeLimit exceeded
    */
    static bool waitOn(std::function< bool(void) >& func, int timeLimit) 
    {
        double startTime = frc::Timer::GetFPGATimestamp().value();
        while (func() || frc::Timer::GetFPGATimestamp().value() - startTime > timeLimit);
        return frc::Timer::GetFPGATimestamp().value() -  startTime > timeLimit;
    }
};