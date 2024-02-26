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
    mArm.zeroSensors();
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
        mIndex.setVelocity(6000.0);
    //     if (!mIndex.isNoteDetected()) {
    //         mIndex.setVelocity(1000);
    //     }
    }
    else if (intakeClear)
    {
        mIndex.setVelocity(-6000.0);
        mIntake.setIntakeState(Intake::CLEAR);
    }
    else
    {
        mIntake.setIntakeState(Intake::STOP);
        mIndex.setVelocity(0.0);
    }
    frc::SmartDashboard::PutBoolean("Intake?", intakeIn);
}

void Superstructure::toggleArm() {
    if (mArm.tiltSetpoint == mArm.highSetpoint) 
    {
        mArm.setPosition(Arm::STOW);
    } else {
        mArm.setPosition(Arm::HIGH);
    }
}

void Superstructure::loadNote()
{
    // Run distance PID on shooter
    // Call this function once(not periodically)
    double distance = 1.0;
    mIndex.setVelocity(500.0);
    mShooter.setDistance(distance);
}

void Superstructure::scoreAmp()
{
    mArm.setPosition(Arm::armPosition::HIGH);
}

void Superstructure::preScoreSpeaker()
{
    mShooter.setSpeed(Shooter::HIGH);
}

void Superstructure::preScoreSpeaker(Limelight limelight) {//find distance to wall using limelight
    // double distanceToWall = limelight.getDistanceToWall();
    // double velocity = mArm.rollerCircumference*1000/60; //converted RPM to meters/second
    // double angle = mArm.findLaunchAngle(velocity, distanceToWall, mArm.speakerHeight);
    mShooter.setSpeed(Shooter::HIGH);
}

void Superstructure::scoreSpeaker()
{
    mIndex.setVelocity(500);
}

void Superstructure::unloadShooter() {
    mShooter.setSpeed(Shooter::LOW);
}

void Superstructure::stow() {
    mArm.setPosition(Arm::STOW);
    mShooter.setSpeed(Shooter::STOP);
}

void Superstructure::updateTelemetry() 
{
    frc::SmartDashboard::PutBoolean("SHOOT?", mShooter.getSpeed() > 4000);
}

void Superstructure::setIndexer(double val) {
    mIndex.indexMotor.Set(val);
}