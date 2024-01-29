#include "Trajectory.h"
#include "SwerveDrive.h"
#include "Robot.h"

#include <frc/controller/HolonomicDriveController.h>
#include <frc/kinematics/ChassisSpeeds.h>


void Trajectory::driveToState(PathPlannerTrajectory::State const &state) 
{

}

void Trajectory::follow(std::string const &traj_dir) 
{
    auto path = PathPlannerPath::fromPathFile("example path");
    PathPlannerTrajectory traj = PathPlannerTrajectory(path, frc::ChassisSpeeds(), frc::Rotation2d(0_rad));

    auto const initialState = traj.getInitialState();
    auto const initialPose = initialState.position; 

    resetOdometry(initialPose, initialState.targetHolonomicRotation); 

    frc::Timer trajTimer; 
    trajTimer.Start(); 

    while (trajTimer.Get() <= traj.getTotalTime()) {
        auto currentTime = trajTimer.Get();
        auto sample = traj.sample(currentTime);

        // use simple auto here to drive to state 
    }

}