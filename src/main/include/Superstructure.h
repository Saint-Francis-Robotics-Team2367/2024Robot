#pragma once

#include <thread>
#include "Intake.h"
#include "Shooter.h"
#include "Index.h"
#include "Arm.h"
#include "sensors/Limelight.h"

class Superstructure
{
private:
    std::thread moduleThread;
    bool enableModules;

public:
    Intake mIntake;
    Shooter mShooter;
    Index mIndex;
    Arm mArm;
    Limelight mLimelight;

    void init();
    void periodic();
    void enable();
    void disable();

    void controlIntake(bool intakeIn, bool intakeClear);
    void preScoreAmp();
    void scoreAmp();
    void preScoreSpeaker();
    void scoreSpeaker();
};