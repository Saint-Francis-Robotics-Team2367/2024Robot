#pragma once

#include <frc/Timer.h>


class TimeDelayedBool 
{
private:
    frc::Timer timer = frc::Timer();
    bool val = false;

public:
    /**
     * Time in seconds
     * Returns true if input has been true for x=time seconds
    */
    bool update(bool input, double time) {
        if (!val && input) {
            timer.Reset();
            timer.Start();
        }
        val = input;
        return (val && (timer.Get().value() >= time));
    }
    




};