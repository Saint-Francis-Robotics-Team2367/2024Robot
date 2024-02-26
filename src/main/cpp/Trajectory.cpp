#include "Trajectory.h"

#include <frc/controller/HolonomicDriveController.h>
#include <frc/kinematics/ChassisSpeeds.h>

// controller used to track trajectories + correct minor disturbances
static frc::HolonomicDriveController controller{
    frc::PIDController{revkP, 0, 0},
    frc::PIDController{revkP, 0, 0},
    frc::ProfiledPIDController<units::radian>{
        steerP, 0, 0,
        frc::TrapezoidProfile<units::radian>::Constraints{
            units::radians_per_second_t(189.2),
            units::radians_per_second_squared_t(2665.993 * (25.8 / 7.6))}}};

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
void Trajectory::follow(std::string const &traj_dir_file_path, bool flipAlliance)
{
    auto path = PathPlannerPath::fromPathFile(traj_dir_file_path);

    // switches path to red alliance (mirrors it)
    if (flipAlliance) {
        path = path->flipPath(); 
    }

    PathPlannerTrajectory traj = PathPlannerTrajectory(path, frc::ChassisSpeeds(), frc::Rotation2d(0_rad));

    auto const initialState = traj.getInitialState();
    auto const initialPose = initialState.position;
    mDrive.resetOdometry(initialPose, initialState.heading);

    frc::Timer trajTimer;
    trajTimer.Start();

    while ((mDrive.state == DriveState::Auto) && (trajTimer.Get() <= traj.getTotalTime()))
    {
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

/**
 * Calls sequences of follow functions for set paths
 * Path naming convention: "[Right Middle Left] Action" 
 */
void Trajectory::followPath(int numPath, bool flipAlliance)
{
    switch (numPath)
    {
    // do nothing
    case 0: 
        break; 

    // straight past stage line
    case 1:
        follow("[R1] Straight", flipAlliance);
        break;

    // [BLUE] 3R - 2 note auto 
    // half court note 5
    case 2:
        mSuperstructure.preScoreSpeaker(); 
        mSuperstructure.scoreSpeaker(); 
        follow("[R] HC 5", flipAlliance);
        mSuperstructure.controlIntake(true, false);
        follow("[R] Score HC 5", flipAlliance);
        mSuperstructure.preScoreSpeaker(); 
        mSuperstructure.scoreSpeaker(); 
        follow("[R] HC 4", flipAlliance); 
        mSuperstructure.controlIntake(true, false);
        break;

    // [BLUE] R - 3 note auto
    // note 3, half court note 5
    case 3:
        mSuperstructure.preScoreSpeaker(); 
        mSuperstructure.scoreSpeaker(); 
        follow("[R] Note 3", flipAlliance);
        mSuperstructure.controlIntake(true, false);
        follow("[R] Score Note 3", flipAlliance);
        mSuperstructure.preScoreSpeaker(); 
        mSuperstructure.scoreSpeaker(); 
        follow("[R] HC 5", flipAlliance); 
        mSuperstructure.preScoreSpeaker(); 
        mSuperstructure.scoreSpeaker(); 
        follow("[R] Score HC 5", flipAlliance); 
        mSuperstructure.preScoreSpeaker(); 
        mSuperstructure.scoreSpeaker(); 
        break;

    // [BLUE] M - 2 note auto 
    // note 2, ends in front of HC 2 
    case 4:
        mSuperstructure.preScoreSpeaker(); 
        mSuperstructure.scoreSpeaker(); 
        follow("[M] Note 2", flipAlliance);
        mSuperstructure.controlIntake(true, false);
        follow("[M] Score Note 2", flipAlliance);
        mSuperstructure.preScoreSpeaker(); 
        mSuperstructure.scoreSpeaker(); 
        follow("[M] HC 2", flipAlliance);
        mSuperstructure.controlIntake(true, false);
        break;

    // [BLUE] L - 2 note auto
    // note 1, ends in front of HC 1
    case 5: 
        mSuperstructure.preScoreSpeaker(); 
        mSuperstructure.scoreSpeaker();
        follow("[L] Note 1", flipAlliance); 
        mSuperstructure.controlIntake(true, false);
        follow("[L] Score Note 1", flipAlliance);
        mSuperstructure.preScoreSpeaker(); 
        mSuperstructure.scoreSpeaker(); 
        follow ("[L] HC 1", flipAlliance); 
        mSuperstructure.controlIntake(true, false);
        break; 

    case 6: 
        mSuperstructure.preScoreSpeaker(); 
        mSuperstructure.scoreSpeaker();
        break; 

    case 7: 
        mSuperstructure.preScoreSpeaker(); 
        mSuperstructure.scoreSpeaker();
        break; 

    case 8: 
        mSuperstructure.preScoreSpeaker(); 
        mSuperstructure.scoreSpeaker();
        break; 

    case 9: 
        break; 

    default:
        break;
    }
}