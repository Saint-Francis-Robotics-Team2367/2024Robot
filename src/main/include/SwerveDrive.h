#pragma once

#include "SwerveModule.h"
#include "geometry/Rotation2d.h"
#include "swerve/ChassisSpeeds.h"
#include "geometry/Pose2d.h"
#include "geometry/Twist2d.h"
#include "geometry/Translation2d.h"
#include "Constants.h"
#include "swerve/SwerveDriveKinematics.h"
#include "swerve/SwerveModuleState.h"
#include <thread>
#include "util/ShuffleUI.h"
#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTableEntry.h>

// Motor/CAN IDs
#define FLsteerID 11
#define FLdriveID 18
#define FL_CAN_ID 3 // updated

#define FRsteerID 4
#define FRdriveID 16
#define FR_CAN_ID 1 // updated

#define BLsteerID 12
#define BLdriveID 10
#define BL_CAN_ID 2 // updated

#define BRsteerID 1
#define BRdriveID 2
#define BR_CAN_ID 0 // updated

// #define maxRot

// TODO: inherit thread helper
class SwerveDrive
{
private:
    SwerveModule mFrontLeft = SwerveModule(FLsteerID, FLdriveID, FL_CAN_ID);
    SwerveModule mFrontRight = SwerveModule(FRsteerID, FRdriveID, FR_CAN_ID);
    SwerveModule mBackLeft = SwerveModule(BLsteerID, BLdriveID, BL_CAN_ID);
    SwerveModule mBackRight = SwerveModule(BRsteerID, BRdriveID, BR_CAN_ID);

    // Threas for each Module
    std::thread modulePIDThread;
    float maxRot = moduleMaxRot;

    // TODO: Rename to Point2d
    std::array<Translation2d, 4> wheelPs = {Translation2d(trackWidth, wheelBase), Translation2d(trackWidth, -wheelBase), Translation2d(-trackWidth, wheelBase), Translation2d(-trackWidth, -wheelBase)};

    SwerveDriveKinematics m_kinematics = SwerveDriveKinematics(wheelPs);

    // Module Level functions
    void runModules(); // Private - do not call outside of init

public:
    // TODO overload - pass Point2d + rotation, it figures it out
    // void Drive(Translation2d translation, Rotation2d rotation);
    void Drive(double rightX, double leftX, double leftY, double fieldRelativeGyro);
    void Drive(ChassisSpeeds desiredSpeeds, Rotation2d fieldRelativeGyro, bool useFieldOriented);
    void initModules();
    void enableModules();
    bool stopModules();
    void orientModules(double FL, double FR, double BL, double BR);
    void autoMove(double angleRadians, double distanceFeet);
    void displayDriveTelemetry();
};
