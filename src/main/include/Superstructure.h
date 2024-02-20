#include <thread>
#include "Intake.h"
#include "Shooter.h"

class Superstructure
{
private:
    std::thread moduleThread;
    bool enableModules;

public:
    Intake mIntake;
    Shooter mShooter;

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