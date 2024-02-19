#include <Superstructure.h>

void Superstructure::init()
{
    mIntake.init();
    mShooter.init();
    enableModules = false;
    moduleThread = std::thread(&Superstructure::periodic, this);
}

void Superstructure::periodic()
{
    while (true)
    {
        if (!enableModules)
        {
            mIntake.disable();
            // Disable modules here
        }
        else
        {
            // PID code here
        }
    }
}

void Superstructure::intakeNote()
{
    mIntake.setIntakeState(Intake::IN);
}

void Superstructure::preScoreAmp()
{
    // Run distance PID on shooter & indexer
}

void Superstructure::scoreAmp()
{
    // once done with distance PID, lift arm up and unload note via shooter
}

void Superstructure::preScoreSpeaker()
{
    // Align shooter & arm to score at speaker
}

void Superstructure::scoreSpeaker()
{
    // Move note into shooter via indexer
}