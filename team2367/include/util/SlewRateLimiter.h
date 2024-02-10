#pragma once

#include <bits/stdc++.h>
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
        previousInput = 0;
        prevTime = frc::Timer::GetFPGATimestamp().value();
    }
    double calculate(double input)
    {
        double currentTime = frc::Timer::GetFPGATimestamp().value();
        double elapsedTime = currentTime - prevTime;
        float range = maxChange * elapsedTime;
        prevTime = currentTime;
        return std::clamp(input, previousInput - range, previousInput + range);
    }
    void reset(double value)
    {
        prevTime = frc::Timer::GetFPGATimestamp().value();
        previousInput = value;
    }
};