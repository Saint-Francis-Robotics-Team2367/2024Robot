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