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
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
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
    
    float indexVelocity = 1200.0;
    if (intakeIn)
    {
        mIntake.setIntakeState(Intake::IN);
        if (!mIndex.isNoteDetected()) 
        {
            
            mIndex.setVelocity(indexVelocity);
        } else {
            mIndex.disable();
        }
        
    //     if (!mIndex.isNoteDetected()) {
    //         mIndex.setVelocity(1000);
    //     }
    }
    else if (intakeClear)
    {
        mIndex.setVelocity(-indexVelocity);
        mIntake.setIntakeState(Intake::CLEAR);
    }
    else
    {
        mIntake.setIntakeState(Intake::STOP);
        mIndex.disable();
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
    // mIndex.setDistance(distance);
    mShooter.setDistance(distance * 2.0 / 3.0);
    mIndex.setDistance(distance * 10);
}

void Superstructure::pushNoteBack() {
    double distance = -1.0;
    // mIndex.setDistance(distance);
    mShooter.setDistance(distance * 2.0 / 3.0);
    mIndex.setDistance(distance * 10);
}

void Superstructure::scoreAmp()
{
    mArm.setPosition(Arm::armPosition::HIGH);
}

void Superstructure::preScoreSpeaker(Pose3d target)
{
    
    mShooter.setSpeed(Shooter::HIGH);
    // frc::SmartDashboard::PutNumber("LimelightX", target.x * 39.37);
    // frc::SmartDashboard::PutNumber("LimelightY", target.y * 39.37);
    double armAngle = mArm.findBetterLaunchAngle(target.x * 39.37, target.y * 39.37, 51.875, 455.59, 18) * 180 / PI;
    // frc::SmartDashboard::PutNumber("TestShooterAngle", armAngle);
    if (armAngle > 0) {
        mArm.setPosition(armAngle);
    }
    
    // mArm.setPosition(60.0);
}

// void Superstructure::preScoreSpeaker(Limelight limelight) {//find distance to wall using limelight
//     // double distanceToWall = limelight.getDistanceToWall();
//     // double velocity = mArm.rollerCircumference*1000/60; //converted RPM to meters/second
//     // double angle = mArm.findLaunchAngle(velocity, distanceToWall, mArm.speakerHeight);
//     mShooter.setSpeed(Shooter::HIGH);
    
// }

void Superstructure::scoreSpeaker()
{
    mIndex.setVelocity(5700);
}

void Superstructure::unloadShooter() {
    mShooter.setSpeed(Shooter::LOW);
}

void Superstructure::stow() {
    mArm.setPosition(Arm::STOW);
    if (!mShooter.inDistanceMode) {
        mShooter.setSpeed(Shooter::STOP);
    }
    
}

void Superstructure::updateTelemetry() 
{
    frc::SmartDashboard::PutBoolean("SHOOT?", mShooter.getSpeed() > 4000);
}

void Superstructure::setIndexer(double val) {
    mIndex.indexMotor.Set(val);
}