#pragma once

#include <frc/Timer.h>
#include <util/ShuffleUI.h>

#define swerveSteerStartCurrent 10
#define swerveDriveStartCurrent 20
#define totalMatchSeconds 150

class PowerModule 
{
    private:
    static bool reduceCurrentsOverTime;


    public:
    static frc::Timer robotTimer;
    static int swerveSteerCurrent;
    static int swerveDriveCurrent;


    static void init(bool enableCurrentCutter);
    static void updateCurrentLimits();

};
