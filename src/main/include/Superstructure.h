#pragma once

#include <thread>
#include "Intake.h"
#include "Shooter.h"
#include "Index.h"
#include "Arm.h"

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

    void init();
    void periodic();
    void enable();
    void disable();

    void controlIntake(bool intakeIn, bool intakeClear);
    void toggleArm();
    void loadNote();
    void pushNoteBack();
    void scoreAmp();
    void preScoreSpeaker(Pose3d target);
    // void preScoreSpeaker(Limelight limelight);
    void scoreSpeaker();
    void unloadShooter();
    void stow();
    void updateTelemetry();
    void setIndexer(double val);
};