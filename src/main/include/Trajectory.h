#pragma once

#include "SwerveDrive.h"
#include "Superstructure.h"
#include "Constants.h"
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
#include "Shooter.h"
#include "sensors/NavX.h"

using namespace pathplanner;

class Trajectory
{
private:
    SwerveDrive &mDrive;
    Superstructure &mSuperstructure; 
    NavX &mGyro;


public:
    Trajectory(SwerveDrive &mDriveInput, Superstructure &mSSInput, NavX &mGyroInput) : mDrive(mDriveInput),
                                                                    mSuperstructure(mSSInput), mGyro(mGyroInput){};

    void driveToState(PathPlannerTrajectory::State const &state, double initialRot);

    void follow(std::string const &traj_dir_file_path, bool flipAlliance);

    void followPath(int numPath, bool flipAlliance);

    void testHolonomic(frc::Pose2d const &target_pose,
                       units::velocity::meters_per_second_t const &velocity,
                       frc::Rotation2d const &target_rot);
};