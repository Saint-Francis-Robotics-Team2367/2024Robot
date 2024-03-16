#pragma once

#include <frc/Timer.h>

class SlewRateLimiter
{
private:
    float maxChange;
    double previousInput;
    double prevTime;

public:
    SlewRateLimiter(float maxChangeInputPerSecond)
    {
        maxChange = maxChangeInputPerSecond;
        previousInput = 0.0;
        prevTime = frc::Timer::GetFPGATimestamp().value();
    }
    double calculate(double input)
    {
        double currentTime = frc::Timer::GetFPGATimestamp().value();
        double elapsedTime = currentTime - prevTime;
        float range = maxChange * elapsedTime;
        prevTime = currentTime;
        
        double output = std::clamp(input, previousInput - range, previousInput + range);
        previousInput = input;
        return output;
    }
    void reset(double value)
    {
        prevTime = frc::Timer::GetFPGATimestamp().value();
        previousInput = value;
    }
};