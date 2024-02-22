#include <Superstructure.h>

void Superstructure::init()
{
    mIntake.init();
    mShooter.init();
    mIndex.init();
    mArm.init();
    enableModules = false;
    moduleThread = std::thread(&Superstructure::periodic, this);
}

void Superstructure::periodic()
{
    while (true)
    {
        if (!enableModules)
        {
            // Disable modules here
            mIntake.disable();
            mIndex.disable();
            mShooter.disable();
            mArm.disable();
        }
        else
        {
            // PID code here
            mArm.runPeriodic();
        }
    }
}

void Superstructure::enable()
{
    enableModules = true;
}

void Superstructure::disable()
{
    enableModules = false;
}

void Superstructure::controlIntake(bool intakeIn, bool intakeClear)
{
    if (intakeIn)
    {
        mIntake.setIntakeState(Intake::IN);
    }
    else if (intakeClear)
    {
        mIntake.setIntakeState(Intake::CLEAR);
    }
    else
    {
        mIntake.setIntakeState(Intake::STOP);
    }
}

void Superstructure::preScoreAmp()
{
    // Run distance PID on shooter & indexer
    // Call this function once(not periodically)
    double distance = 3.0;
    mIndex.setDistance(distance);
    mShooter.setDistance(distance);
}

void Superstructure::scoreAmp()
{
    // once done with distance PID, lift arm up and unload note via shooter
    bool preScoringDone = mIndex.isDistanceFinished(0.05) && mShooter.isDistanceFinished(0.05);
    if (preScoringDone)
    {
        mArm.setPosition(Arm::armPosition::HIGH);
    }
    
}

void Superstructure::preScoreSpeaker()
{
    // Align shooter & arm to score at speaker
    mShooter.setSpeed(5700);
    mArm.setPosition(0);
    mArm.runPeriodic();
}

void Superstructure::scoreSpeaker()
{
    // Move note into shooter via indexer
    mIndex.setDistance(5);
    
}