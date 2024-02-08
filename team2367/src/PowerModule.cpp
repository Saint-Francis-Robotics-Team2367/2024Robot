#include "control/PowerModule.h"



int PowerModule::swerveDriveCurrent;
int PowerModule::swerveSteerCurrent;
frc::Timer PowerModule::robotTimer;
bool PowerModule::reduceCurrentsOverTime;

void PowerModule::init(bool enableCurrentCutter) 
{
    PowerModule::robotTimer.Reset();
    PowerModule::robotTimer.Start();
    PowerModule::reduceCurrentsOverTime = enableCurrentCutter;
    PowerModule::swerveDriveCurrent = swerveDriveStartCurrent;
    PowerModule::swerveSteerCurrent = swerveSteerStartCurrent;
}

void PowerModule::updateCurrentLimits()
{
    int cutAmt;
    if (PowerModule::reduceCurrentsOverTime);
    {
        cutAmt = 5 * floor(robotTimer.Get().value() / 50);
        
        PowerModule::swerveDriveCurrent = swerveDriveStartCurrent - cutAmt;
        if (PowerModule::swerveDriveCurrent < 5) 
        {
            swerveDriveCurrent = 5;
        }
    }
    ShuffleUI::MakeWidget("SwerveDrive Current", "Power", swerveDriveCurrent);
    ShuffleUI::MakeWidget("Timer", "Power", robotTimer.Get().value());
    ShuffleUI::MakeWidget("CutAmt", "Power", cutAmt);
}
