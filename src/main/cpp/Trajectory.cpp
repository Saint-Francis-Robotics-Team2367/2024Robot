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
        steerP, 0, 0,
        frc::TrapezoidProfile<units::radian>::Constraints{
            units::radians_per_second_t(SwerveModule::maxSteerVelocity),
            units::radians_per_second_squared_t(SwerveModule::maxDriveAccelerationRadPS)}}}; 


/**
 * Drives robot to the next state on trajectory
 * Odometry must be in meters
 */
void Trajectory::driveToState(PathPlannerTrajectory::State const &state) 
{
    frc::ChassisSpeeds const correction = controller.Calculate(mDrive.getOdometryPose(), frc::Pose2d{state.position, state.heading}, state.velocity, state.targetHolonomicRotation);

    double vx_feet = correction.vx.value() * 3.281; 
    double vy_feet = correction.vy.value() * 3.281;
    double rotCompass = Rotation2d::compassToPolar(state.targetHolonomicRotation.Radians().value());
    
    mDrive.Drive(ChassisSpeeds{-vy_feet, vx_feet, correction.omega.value()}, Rotation2d{rotCompass}, false);

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
    mDrive.resetOdometry(initialPose, initialState.heading); 

    frc::Timer trajTimer; 
    trajTimer.Start(); 

    while ((mDrive.state == 'a') && (trajTimer.Get() <= traj.getTotalTime())) {
        auto currentTime = trajTimer.Get();
        auto sample = traj.sample(currentTime);

        driveToState(sample);
        mDrive.updateOdometry(); 

        frc::SmartDashboard::PutNumber("curr pose x meters", mDrive.getOdometryPose().Translation().X().value());
        frc::SmartDashboard::PutNumber("curr pose y meters", mDrive.getOdometryPose().Translation().Y().value());


        using namespace std::chrono_literals;

        // refresh rate of holonomic drive controller's PID controllers (edit if needed)
        std::this_thread::sleep_for(20ms); 

    }
    mDrive.stopModules();

}