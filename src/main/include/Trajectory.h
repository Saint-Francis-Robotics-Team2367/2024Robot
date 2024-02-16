#pragma once

#include "SwerveDrive.h"
#include "SwerveModule.h"
#include "Constants.h"
#include "Shooter.h"
#include "Robot.h"
#include "sensors/Limelight.h"
#include "geometry/Translation2d.h"

#include <pathplanner/lib/path/PathPlannerTrajectory.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/path/GoalEndState.h>

#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Pose2d.h"

#include "units/velocity.h"
#include "units/angle.h"
#include "units/angular_acceleration.h"
#include "units/angular_velocity.h"

#include <chrono>
#include <frc/Timer.h>

using namespace pathplanner;

class Trajectory
{
private:
    SwerveDrive &mDrive;
    Shooter &mShooter;
    Limelight &mLimelight;

public:
    Trajectory(SwerveDrive &mDriveInput, Shooter &mShooterInput, Limelight &mLimelightInput) : mDrive(mDriveInput),
                                                                                               mShooter(mShooterInput),
                                                                                               mLimelight(mLimelightInput){};

    void driveToState(PathPlannerTrajectory::State const &state);

    void follow(std::string const &traj_dir);

    void followPath(int num);

    void testHolonomic(frc::Pose2d const &target_pose,
                       units::velocity::meters_per_second_t const &velocity,
                       frc::Rotation2d const &target_rot);
};