
#include "SwerveDrive.h"
#include <cmath>
#include <string>

/*
 * Left Stick controls robot velocity(direction & speed)
 * Right Stick X controls robot rotational speed
 * Gyro is used to make the robot drive field-centric
 */

void SwerveDrive::Drive(double rightX, double leftX, double leftY, double fieldRelativeGyro)
{
    ShuffleUI::MakeWidget("rightX", "CTR", rightX);
    ShuffleUI::MakeWidget("leftY", "CTR", leftY);
    ShuffleUI::MakeWidget("leftX", "CTR", leftX);

    if ((leftY == 0) && (leftX == 0) && (rightX == 0)) 
    {
        mFrontLeft.setDriveVelocitySetpoint(0.0);
        mFrontRight.setDriveVelocitySetpoint(0.0);
        mBackLeft.setDriveVelocitySetpoint(0.0);
        mBackRight.setDriveVelocitySetpoint(0.0);
        return;
    }



    // Creating desired Chassis speeds from controller input
    double Vx = leftX * moduleMaxFPS;
    double Vy = leftY * moduleMaxFPS;
    double omega = rightX * maxRot;

    // Choose between field-centric & robot-centric
    /*
    Note: FieldRelativeGyro should be taken in the compass reference system
    However, kinematics module still outputs in polar reference system
    */



    ChassisSpeeds desiredSpeeds = ChassisSpeeds::fromFieldRelativeSpeeds(Vx, Vy, omega, fieldRelativeGyro);
    // ChassisSpeeds desiredSpeeds = ChassisSpeeds(Vx, Vy, omega);


    // Feeding chassis speeds into kinematics module(which works, I tested it)
    std::vector<SwerveModuleState> moduleStates = m_kinematics.toSwerveStates(desiredSpeeds);

    // Printing the setpoints for our single module
    // BTW order of motors is FL, FR, BL, BR so [2] corresponds to BL

    /**
     * Kinematics class returns module orientations in polar degrees
     * This means that 0 degrees is "to the right"
     * We want 0 degrees to be forward, so we use convertAngleReference()
     * Kinematics class also returns module speeds in ft/sec
     * We need to convert back FPS to RPM for the PIDs, so we use our conversion factors
     * FPS * 60 = FPM(feet per min)
     * FPM / (circum) = wheel rotations per minute(WPM)
     * WPM * (6.12 motor rot / 1 wheel rot) = RPM
     */

    for (int i = 0; i < 4; i++)
    {
        double speed = moduleStates[i].getSpeedFPS();
        speed = ((speed * 60) / wheelCircumFeet) * moduleDriveRatio;
        SwerveModuleState temp = SwerveModuleState(speed, moduleStates[i].getRot2d().getRadians());
        moduleStates[i] = temp;

        frc::SmartDashboard::PutNumber(std::to_string(i) + "vel", speed);
        frc::SmartDashboard::PutNumber(std::to_string(i) + "angle", moduleStates[i].getRot2d().getDegrees());

    }

    // Order of kinematics output is always FL, FR, BL, BR
    mFrontLeft.setModuleState(moduleStates[1]);
    mFrontRight.setModuleState(moduleStates[3]);
    mBackLeft.setModuleState(moduleStates[0]);
    mBackRight.setModuleState(moduleStates[2]);
    
}

void SwerveDrive::setModuleVelocity(SwerveModule &mModule, double speed, double angleRadians)
{
    mModule.setDriveVelocitySetpoint(speed);
    mModule.setSteerAngleSetpoint(angleRadians);
}

/**
 * Initialize every motor(encoders, factory reset, current limits, etc)
 * Initialize each motor thread, which should start the threads
 */
void SwerveDrive::initAllMotors()
{
    mFrontLeft.initMotors();
    mFrontRight.initMotors();
    mBackLeft.initMotors();
    mBackRight.initMotors();

    modulePIDThread = std::thread(&SwerveDrive::runModules, this);
}
/**
 * Do not call this code outside of initAllMotors's thread
*/
void SwerveDrive::runModules() 
{
    while (true) {
        mFrontLeft.run();
        mFrontRight.run();
        mBackLeft.run();
        mBackRight.run();
    }
    
}


/**
 * Set every module's threads to active mode
 * So the PIDs start running
 */
void SwerveDrive::enableThreads()
{
    mFrontLeft.exitStandbyThread();
    mBackLeft.exitStandbyThread();
    mBackRight.exitStandbyThread();
    mFrontRight.exitStandbyThread();
}
/**
 * Disable every module's thread
 * Threads still exist, just on standby while loop
 */
bool SwerveDrive::stopAllMotors()
{
    mFrontLeft.standbyThread();
    mBackLeft.standbyThread();
    mBackRight.standbyThread();
    mFrontRight.standbyThread();
    return true;
}

/**
 * Enter radians
 * Converts from zero = right to zero = forward
 * Also inverts the angle
 */
double SwerveDrive::convertAngleReference(double angle)
{
    angle = -angle + M_PI_2;
    angle = angle * 180 / M_PI;
    angle = fmod(angle + 360, 360);
    angle = angle * M_PI / 180;

    return angle;
}
/**
 * Enter radians
 * Sets steer Angle setpoint to inputs
 */
void SwerveDrive::orientModules(double FL, double FR, double BL, double BR)
{
    mBackRight.setSteerAngleSetpoint(BR);
    mBackLeft.setSteerAngleSetpoint(BL);
    mFrontRight.setSteerAngleSetpoint(FR);
    mFrontLeft.setSteerAngleSetpoint(FL);
}

/**
 * Incomplete function
 * Awaits movement
*/
void SwerveDrive::autoMove(double angleRadians, double distanceFeet) 
{
    orientModules(angleRadians, angleRadians, angleRadians, angleRadians);
    //Wait for modules w/ while loop
    mFrontLeft.setDrivePositionSetpoint(distanceFeet);
    mFrontRight.setDrivePositionSetpoint(distanceFeet);
    mBackLeft.setDrivePositionSetpoint(distanceFeet);
    mBackRight.setDrivePositionSetpoint(distanceFeet);
    //Wait for modules to reach point
}

/**
 * Uses shuffleUI to print to driveTab
 * Uses gyro widget
 * Flips angle gyro if module has negative velocity
*/
void SwerveDrive::displayDriveTelemetry() 
{
    // All shuffleboard prints must have a defined place
    //mFrontLeft.displayModuleState("FrontLeft", 0, 0);
    double FR = mFrontRight.getSteerEncoder().getDegrees();
    double FL = mFrontLeft.getSteerEncoder().getDegrees();
    double BR = mBackRight.getSteerEncoder().getDegrees();
    double BL = mBackLeft.getSteerEncoder().getDegrees();

    frc::SmartDashboard::PutNumber("FL", FL);
    frc::SmartDashboard::PutNumber("FR", FR);
    frc::SmartDashboard::PutNumber("BL", BL);
    frc::SmartDashboard::PutNumber("BR", BR);

    // FRentry->SetDouble(FL);

    // ShuffleUI::MakeWidget("FL", "drive", FL, frc::BuiltInWidgets::kGyro, 0, 0);
    // ShuffleUI::MakeWidget("FR", "drive", FR, frc::BuiltInWidgets::kGyro, 0, 2);
    // ShuffleUI::MakeWidget("BL", "drive", BL, frc::BuiltInWidgets::kGyro, 2, 0);
    // ShuffleUI::MakeWidget("BR", "drive", BR, frc::BuiltInWidgets::kGyro, 2, 2);

    // FRentry.GetEntry()->SetDouble(FR.getRot2d().getDegrees());
    // ShuffleUI::MakeWidget("FR", driveTab, FR.getRot2d().getDegrees(), frc::BuiltInWidgets::kGyro);
    //mBackLeft.displayModuleState("BackLeft", 2, 0);
    // mBackRight.displayModuleState("BackRight", 2, 2);
    //ShuffleUI::PrintWidgetList();

    // ShuffleUI::MakeWidget("FLsteerOut", "drive", mFrontLeft.getSteerOutput(), 0, 4);
    // ShuffleUI::MakeWidget("FRsteerOut", "drive", mFrontRight.getSteerOutput(), 0, 5);
    // ShuffleUI::MakeWidget("BLsteerOut", "drive", mBackLeft.getSteerOutput(), 1, 4);
    // ShuffleUI::MakeWidget("BRsteerOut", "drive", mBackRight.getSteerOutput(), 1, 5);
    

    // ShuffleUI::MakeWidget("driveMode", driveTab, driveMode, 3, 0);

}