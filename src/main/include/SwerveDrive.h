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
#include "sensors/NavX.h"

#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/kinematics/SwerveDriveOdometry.h"
#include <pathplanner/lib/path/PathPlannerTrajectory.h>
#include <pathplanner/lib/path/PathPlannerPath.h>

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
#define BRdriveID 42
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

    NavX m_NavX = NavX(); 

    // wpi lib class ver of kinemactics used to initialize odometry
    frc::SwerveDriveKinematics<4> frckinematics{ 
        frc::Translation2d{0.7239_m, 0.7239_m},
        frc::Translation2d{0.7239_m, -0.7239_m},
        frc::Translation2d{-0.7239_m, 0.7239_m},
        frc::Translation2d{-0.7239_m, -0.7239_m},
    };

    frc::SwerveDriveOdometry<4> m_odometry{
        frckinematics,
        m_NavX.getRotation2d(), 
        // might need to edit order of motors (double check)
        {
            mBackLeft.getModulePosition(), 
            mFrontLeft.getModulePosition(), 
            mFrontRight.getModulePosition(), 
            mBackRight.getModulePosition() 
        },
        frc::Pose2d{0_m, 0_m, 0_deg}
    };

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
    void resetOdometry(frc::Translation2d trans, frc::Rotation2d angle);
    void displayDriveTelemetry();
};
