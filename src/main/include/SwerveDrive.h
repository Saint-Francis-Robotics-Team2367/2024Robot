#pragma once

#include "SwerveModule.h"
#include "geometry/Rotation2d.h"
#include "swerve/ChassisSpeeds.h"
#include "geometry/Translation2d.h"
#include "Constants.h"
#include "swerve/SwerveDriveKinematics.h"
#include "swerve/SwerveModuleState.h"
#include <thread>
#include "util/ShuffleUI.h"
#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTableEntry.h>

class SwerveDrive
{
private:

    SwerveModule mFrontLeft = SwerveModule(FLsteerID, FLdriveID, FL_CAN_ID);
    SwerveModule mFrontRight = SwerveModule(FRsteerID, FRdriveID, FR_CAN_ID);
    SwerveModule mBackLeft = SwerveModule(BLsteerID, BLdriveID, BL_CAN_ID);
    SwerveModule mBackRight = SwerveModule(BRsteerID, BRdriveID, BR_CAN_ID);

    
    // Threas for each Module
    std::thread modulePIDThread;

    //float maxSpeed = moduleMaxFPS; // feet/sec, since 5700 RPM = 16 ft/s * 356.25, we have conversion factor
    float maxRot = moduleMaxRot;

    // Kinematic module: wheelPs creates x,y coordinates for each module with 0,0 being center of the robot
    std::vector<Translation2d> wheelPs = {Translation2d(trackWidth, wheelBase), Translation2d(trackWidth, -wheelBase), Translation2d(-trackWidth, wheelBase), Translation2d(-trackWidth, -wheelBase)};
    SwerveDriveKinematics m_kinematics = SwerveDriveKinematics(wheelPs);

    int driveMode = 0;
    // nt::NetworkTableEntry entry = frc::Shuffleboard::GetTab(driveTab).Add
    


    // Module Level functions
    void runModules(); // Private - do not call outside of init

public:
    void Drive(double rightX, double leftX, double leftY, double fieldRelativeGyro);
    void setModuleVelocity(SwerveModule &mModule, double speed, double angleRadians);
    void initAllMotors();
    void enableThreads();
    bool stopAllMotors();
    double convertAngleReference(double input);
    void orientModules(double FL, double FR, double BL, double BR);
    void autoMove(double angleRadians, double distanceFeet);
    void displayDriveTelemetry();
    // void autoRotate(double angleRadians); Needs to be thought out a little
};
