
#include "SwerveDrive.h"
#include <cmath>
#include <string>
#include <chrono>
#include <thread>

/*
 * Left Stick controls robot velocity(direction & speed)
 * Right Stick X controls robot rotational speed
 * Gyro is used to make the robot drive field-centric
 */

void SwerveDrive::Drive(double rightX, double leftX, double leftY, double fieldRelativeGyro)
{
    if ((leftY == 0) && (leftX == 0) && (rightX == 0))
    {

        SwerveModuleState FLBRstop = SwerveModuleState(0.0, M_PI / 4);
        SwerveModuleState FRBLstop = SwerveModuleState(0.0, 7 * M_PI / 4);

        // mFrontLeft.setModuleState(FLBRstop, false);
        // mFrontRight.setModuleState(FRBLstop, false);
        // mBackLeft.setModuleState(FLBRstop, false);
        // mBackRight.setModuleState(FRBLstop, false);
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
    moduleStates = m_kinematics.desaturateWheelSpeeds(moduleStates, moduleMaxFPS);

    // Printing the setpoints for our single module
    // BTW order of motors is FL, FR, BL, BR so [2] corresponds to BL

    /**
     * Kinematics class returns module orientations in polar degrees
     * This means that 0 degrees is "to the right"
     * We want 0 degrees to be forward
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

        // frc::SmartDashboard::PutNumber(std::to_string(i) + "vel", speed);
        // frc::SmartDashboard::PutNumber(std::to_string(i) + "angle", moduleStates[i].getRot2d().getDegrees());
    }

    // Order of kinematics output is always FL, FR, BL, BR
    mFrontLeft.setModuleState(moduleStates[1], true);
    mFrontRight.setModuleState(moduleStates[3], true);
    mBackLeft.setModuleState(moduleStates[0], true);
    mBackRight.setModuleState(moduleStates[2], true);
}

void SwerveDrive::Drive(ChassisSpeeds desiredSpeeds, Rotation2d fieldRelativeGyro, bool useFieldOriented)
{
    double desiredVx = desiredSpeeds.vxMetersPerSecond; // FEET PER SECOND
    double desiredVy = desiredSpeeds.vyMetersPerSecond;

    if (fabs(desiredVx) < kEpsilon && fabs(desiredVy) < kEpsilon && fabs(desiredSpeeds.omegaRadiansPerSecond) < kEpsilon) 
    {
        SwerveModuleState FLBRstop = SwerveModuleState(0.0, M_PI / 4);
        SwerveModuleState FRBLstop = SwerveModuleState(0.0, 7 * M_PI / 4);

        mFrontLeft.setModuleState(FLBRstop, true);
        mFrontRight.setModuleState(FRBLstop, true);
        mBackLeft.setModuleState(FRBLstop, true);
        mBackRight.setModuleState(FLBRstop, true);

        // mFrontLeft.setDriveVelocitySetpoint(0.0);
        // mFrontRight.setDriveVelocitySetpoint(0.0);
        // mBackLeft.setDriveVelocitySetpoint(0.0);
        // mBackRight.setDriveVelocitySetpoint(0.0);
        return;
    }

    if (useFieldOriented) {
        desiredSpeeds = ChassisSpeeds::fromFieldRelativeSpeeds(desiredVx, desiredVy, desiredSpeeds.omegaRadiansPerSecond, fieldRelativeGyro);
        frc::SmartDashboard::PutBoolean("BOTCENTRIC!", false);
    } else {
        frc::SmartDashboard::PutBoolean("BOTCENTRIC!", true);
    }

    std::vector<SwerveModuleState> moduleStates = m_kinematics.toSwerveStates(desiredSpeeds);
    moduleStates = m_kinematics.desaturateWheelSpeeds(moduleStates, moduleMaxFPS);
    /**
     * Kinematics class returns module orientations in polar degrees
     * This means that 0 degrees is "to the right"
     * We want 0 degrees to be forward
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

        // frc::SmartDashboard::PutNumber(std::to_string(i) + "vel", speed);
        // frc::SmartDashboard::PutNumber(std::to_string(i) + "angle", moduleStates[i].getRot2d().getDegrees());
    }

    // Order of kinematics output is always FL, FR, BL, BR
    mFrontLeft.setModuleState(moduleStates[1], true);
    mFrontRight.setModuleState(moduleStates[3], true);
    mBackLeft.setModuleState(moduleStates[0], true);
    mBackRight.setModuleState(moduleStates[2], true);
}

/**
 * Initialize every motor(encoders, factory reset, current limits, etc)
 * Initialize each motor thread, which should start the threads
 */
void SwerveDrive::initModules()
{
    mFrontLeft.initMotors();
    mFrontRight.initMotors();
    mBackLeft.initMotors();
    mBackRight.initMotors();

    modulePIDThread = std::thread(&SwerveDrive::runModules, this);
}
/**
 * Do not call this code outside of initModules's thread
 */
void SwerveDrive::runModules()
{
    while (true)
    {
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
void SwerveDrive::enableModules()
{
    mFrontLeft.startModule();
    mBackLeft.startModule();
    mBackRight.startModule();
    mFrontRight.startModule();
}
/**
 * Disable every module's thread
 * Threads still exist, just on standby while loop
 */
bool SwerveDrive::stopModules()
{
    mFrontLeft.stopModule();
    mBackLeft.stopModule();
    mBackRight.stopModule();
    mFrontRight.stopModule();
    return true;
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

/**x
 * Incomplete function
 * Awaits movement
 */
void SwerveDrive::autoMove(double angleRadians, double distanceFeet)
{
    orientModules(angleRadians, angleRadians, angleRadians, angleRadians);
    // TODO: Wait for modules w/ while loop
    mFrontLeft.setDrivePositionSetpoint(distanceFeet);
    mFrontRight.setDrivePositionSetpoint(distanceFeet);
    mBackLeft.setDrivePositionSetpoint(distanceFeet);
    mBackRight.setDrivePositionSetpoint(distanceFeet);
    // TODO: Wait for modules to reach point
}

/**
 * Resets odometry position 
 * (used in auto config)
*/
void SwerveDrive::resetOdometry(frc::Translation2d trans, frc::Rotation2d rot) {
    m_odometry.ResetPosition(
        mGyro.getRotation2d(), 
        {
            mBackLeft.getModulePosition(), 
            mFrontLeft.getModulePosition(), 
            mFrontRight.getModulePosition(), 
            mBackRight.getModulePosition() 
        },
        frc::Pose2d{trans, rot}
    );
    
}

/**
 * Retrieves odometry pose in feet 
*/
frc::Pose2d SwerveDrive::getOdometryPose()
{
    return m_odometry.GetPose();
}

/**
 * Updates odometry with current module positions 
*/
void SwerveDrive::updateOdometry() 
{
    m_odometry.Update(
        -mGyro.getRotation2d(), 
        {
            mBackLeft.getModulePosition(), 
            mFrontLeft.getModulePosition(), 
            mFrontRight.getModulePosition(), 
            mBackRight.getModulePosition() 
        }
    );
}

/**
 * Uses shuffleUI to print to driveTab
 * Uses gyro widget
 * Flips angle gyro if module has negative velocity
 */
void SwerveDrive::displayDriveTelemetry()
{
    
}