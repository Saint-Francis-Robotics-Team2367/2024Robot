#include "Trajectory.h"
#include "SwerveDrive.h"
#include "SwerveModule.h"
#include "Robot.h"

#include <frc/controller/HolonomicDriveController.h>
#include <frc/kinematics/ChassisSpeeds.h>



// controller used to track trajectories + correct minor disturbances 
static frc::HolonomicDriveController controller{
    frc::PIDController{revkP, 0, 0},
    frc::PIDController{revkP, 0, 0},
    frc::ProfiledPIDController<units::radian>{
        10, -0.003, 0,
        frc::TrapezoidProfile<units::radian>::Constraints{
            units::radians_per_second_t(SwerveModule::maxSteerVelocity),
            units::radians_per_second_squared_t(SwerveModule::maxDriveAccelerationRPM)}}}; // need max angular acceleration 


/**
 * Drives robot to the next state on trajectory
 */
void Trajectory::driveToState(PathPlannerTrajectory::State const &state) 
{
    frc::ChassisSpeeds const correction = controller.Calculate(mDrive.getOdometryPose(), frc::Pose2d{state.position, state.heading}, state.velocity, state.targetHolonomicRotation);
    mDrive.Drive(ChassisSpeeds{correction.vx.value(), correction.vy.value(), correction.omega.value()},Rotation2d{state.targetHolonomicRotation.Radians().value()}, false);

}

/**
 * Follows pathplanner trajectory
*/
void Trajectory::follow(std::string const &traj_dir) 
{
    auto path = PathPlannerPath::fromPathFile(traj_dir);
    PathPlannerTrajectory traj = PathPlannerTrajectory(path, frc::ChassisSpeeds(), frc::Rotation2d(0_rad));

    auto const initialState = traj.getInitialState();
    auto const initialPose = initialState.position; 

    mDrive.resetOdometry(initialPose, initialState.targetHolonomicRotation); 

    frc::Timer trajTimer; 
    trajTimer.Start(); 

    while ((mDrive.state == 'a') && (trajTimer.Get() <= traj.getTotalTime())) {
        auto currentTime = trajTimer.Get();
        auto sample = traj.sample(currentTime);

        driveToState(sample);
        mDrive.updateOdometry(); 

        // note: odometry values are in feet; pathplanner values are in meters
        frc::SmartDashboard::PutNumber("curr pose x", mDrive.getOdometryPose().Translation().X().value());
        frc::SmartDashboard::PutNumber("curr pose y", mDrive.getOdometryPose().Translation().X().value());


        using namespace std::chrono_literals;

        // refresh rate of holonomic drive controller's PID controllers (edit if needed)
        std::this_thread::sleep_for(20ms); 

    }
    mDrive.stopModules();

}