#include "SwerveDrive.h"
#include <cmath>
#include <string>
#include <chrono>
#include <thread>

void SwerveDrive::Drive(ChassisSpeeds desiredSpeeds, Rotation2d fieldRelativeGyro, bool useFieldOriented, bool cleanAccum)
{
    double desiredVx = desiredSpeeds.vxMetersPerSecond;
    double desiredVy = desiredSpeeds.vyMetersPerSecond;
    if (useFieldOriented)
    {
        desiredSpeeds = ChassisSpeeds::fromFieldRelativeSpeeds(desiredVx, desiredVy, desiredSpeeds.omegaRadiansPerSecond, fieldRelativeGyro);
        frc::SmartDashboard::PutBoolean("BOTCENTRIC!", false);
    }
    else
    {
        frc::SmartDashboard::PutBoolean("BOTCENTRIC!", true);
    }
    desiredVx = desiredSpeeds.vxMetersPerSecond;
    desiredVy = desiredSpeeds.vyMetersPerSecond;

    if (cleanAccum && fabs(desiredVx) < (moduleMaxFPS * 0.1) && fabs(desiredVy) < (moduleMaxFPS * 0.1)) {
        zeroAccumulation();
        frc::SmartDashboard::PutNumber("CleanedAccum", true);
    } else {
        frc::SmartDashboard::PutNumber("CleanedAccum", false);
    }

    if (fabs(desiredVx) < kEpsilon && fabs(desiredVy) < kEpsilon && fabs(desiredSpeeds.omegaRadiansPerSecond) < kEpsilon)
    {
        // SwerveModuleState FLBRstop = SwerveModuleState(0.0, PI / 4);
        // SwerveModuleState FRBLstop = SwerveModuleState(0.0, 7 * PI / 4);

        // mFrontLeft.setModuleState(FLBRstop, true);
        // mFrontRight.setModuleState(FRBLstop, true);
        // mBackLeft.setModuleState(FRBLstop, true);
        // mBackRight.setModuleState(FLBRstop, true);

        mFrontLeft.setDriveVelocitySetpoint(0.0);
        mFrontRight.setDriveVelocitySetpoint(0.0);
        mBackLeft.setDriveVelocitySetpoint(0.0);
        mBackRight.setDriveVelocitySetpoint(0.0);
        return;
    }

    // Pose2d robotPoseVel = Pose2d(desiredVx * loopTime, desiredVy * loopTime, Rotation2d(desiredSpeeds.omegaRadiansPerSecond * loopTime));
    // Twist2d robotTwist = Pose2d::log(robotPoseVel);
    // ChassisSpeeds newDesiredSpeeds = ChassisSpeeds(robotTwist.dx / loopTime, robotTwist.dy / loopTime, robotTwist.dtheta / loopTime);
    // ShuffleUI::MakeWidget("Xspeed", "drive", newDesiredSpeeds.vxMetersPerSecond);
    // ShuffleUI::MakeWidget("Yspeed", "drive", newDesiredSpeeds.vyMetersPerSecond);
    // ShuffleUI::MakeWidget("Rot", "drive", newDesiredSpeeds.omegaRadiansPerSecond);

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
        // SwerveModuleState temp = SwerveModuleState(speed, moduleStates[i].getRot2d().getRadians());
        moduleStates[i].setSpeedFPS(speed);
        // moduleStates[i] = temp;

        // frc::SmartDashboard::PutNumber(std::to_string(i) + "vel", speed);
        // frc::SmartDashboard::PutNumber(std::to_string(i) + "angle", moduleStates[i].getRot2d().getDegrees());
    }

    // Order of kinematics output is always BL, FL, BR, FR
    mFrontLeft.setModuleState(moduleStates[1], true);
    mFrontRight.setModuleState(moduleStates[3], true);
    mBackLeft.setModuleState(moduleStates[0], true);
    mBackRight.setModuleState(moduleStates[2], true);

    frc::SmartDashboard::PutNumber("desired speeds vx", desiredSpeeds.vxMetersPerSecond);
    frc::SmartDashboard::PutNumber("desired speeds vy", desiredSpeeds.vyMetersPerSecond);
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
        std::this_thread::sleep_for(std::chrono::milliseconds(12));
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

// void SwerveDrive::setDriveCurrentLimit(int limit)
// {
//     mFrontRight.setDriveCurrentLimit(limit);
//     mFrontLeft.setDriveCurrentLimit(limit);
//     mBackLeft.setDriveCurrentLimit(limit);
//     mBackRight.setDriveCurrentLimit(limit);
// }

/**
 * Resets odometry position
 * (used in auto config)
 */
void SwerveDrive::resetOdometry(frc::Translation2d trans, frc::Rotation2d angle)
{
    m_odometry.ResetPosition(
        mGyro.getRotation2d(),
        {mBackLeft.getModulePosition(),
         mFrontLeft.getModulePosition(),
         mFrontRight.getModulePosition(),
         mBackRight.getModulePosition()},
        frc::Pose2d{trans, angle});
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
        {mBackLeft.getModulePosition(),
         mFrontLeft.getModulePosition(),
         mFrontRight.getModulePosition(),
         mBackRight.getModulePosition()});
}

/**
 * Uses shuffleUI to print to driveTab
 * Uses gyro widget
 * Flips angle gyro if module has negative velocity
 */
void SwerveDrive::displayDriveTelemetry()
{
}

void SwerveDrive::zeroAccumulation() {
    // mFrontLeft.m_pidController.SetIAccum(0.0);
    // mFrontRight.m_pidController.SetIAccum(0.0);
    // mBackRight.m_pidController.SetIAccum(0.0);
    // mBackLeft.m_pidController.SetIAccum(0.0);
}