#pragma once

#include <frc/Timer.h>

class TimeDelayButton 
{
    // if button is pressed return true after press for x seconds
private:
    frc::Timer timer = frc::Timer();
    bool wasPressed = false;

public:
    bool update(bool value, float delayTimeSeconds) 
    {
        if (value) 
        {
            // Button pressed, start timer
            timer.Reset();
            timer.Start();
            wasPressed = true;
            return value;
        } else if ((timer.Get().value() < delayTimeSeconds) && wasPressed) 
        {
            // Button was pressed, timer is in time
            return true;
        } else 
        {
            // Button hasn't been pressed or timer went over
            wasPressed = false;
            return false;
        }

    }

};