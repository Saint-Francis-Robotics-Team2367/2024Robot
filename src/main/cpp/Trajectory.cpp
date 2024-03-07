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
void Trajectory::driveToState(PathPlannerTrajectory::State const &state, double initialRot)
{
    // Calculate new chassis speeds given robot position and next desired state in trajectory
    frc::ChassisSpeeds const correction = controller.Calculate(mDrive.getOdometryPose(), frc::Pose2d{state.position, state.heading}, state.velocity, state.targetHolonomicRotation);

    // Calculate x, y speeds from MPS
    double vx_feet = correction.vx.value() * 3.281;
    double vy_feet = correction.vy.value() * 3.281;

    // Clamp rot speed to 2.0 since that is the max rot we allow
    double rot = std::clamp(correction.omega.value(), -2.0, 2.0);

    frc::SmartDashboard::PutNumber("VY", vy_feet);
    frc::SmartDashboard::PutNumber("VX", vx_feet);

    mDrive.Drive(ChassisSpeeds{-vy_feet, vx_feet, rot}, mGyro.getRotation2d(initialRot).Degrees().value(), true, true);
}

/**
 * Follows pathplanner trajectory
 */
void Trajectory::follow(std::string const &traj_dir_file_path, bool flipAlliance)
{
    mDrive.enableModules();
    auto path = PathPlannerPath::fromPathFile(traj_dir_file_path);

    // switches path to red alliance (mirrors it)
    if (flipAlliance) {
        path = path->flipPath(); 
    }

    PathPlannerTrajectory traj = PathPlannerTrajectory(path, frc::ChassisSpeeds(), frc::Rotation2d(0_rad));

    auto const initialState = traj.getInitialState();
    auto const initialPose = initialState.position;
    auto const initialRot = traj.getInitialTargetHolonomicPose().Rotation().Degrees().value(); 
    mDrive.resetOdometry(initialPose, initialState.heading, initialRot);

    frc::Timer trajTimer;
    trajTimer.Start();

    while ((mDrive.state == DriveState::Auto) && (trajTimer.Get() <= traj.getTotalTime()))
    {
        auto currentTime = trajTimer.Get();
        auto sample = traj.sample(currentTime);

        driveToState(sample, initialRot);
        mDrive.updateOdometry(initialRot);

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
    // std::this_thread::sleep_for(std::chrono::seconds(7));

    mSuperstructure.mShooter.setSpeed(Shooter::HIGH);
    // Wait until shooter reaches 4000 RPM or 3 seconds pass
    double startTimeShooter = frc::Timer::GetFPGATimestamp().value();
    while (mSuperstructure.mShooter.getSpeed() < 4000 || frc::Timer::GetFPGATimestamp().value() - startTimeShooter > 3.0) {};
    // Run indexer
    mSuperstructure.mIndex.setVelocity(2000.0);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // Stop Shooter

    mSuperstructure.mIndex.setVelocity(0.0);
    // mSuperstructure.mShooter.setSpeed(Shooter::STOP);
    mSuperstructure.controlIntake(true, false);

    follow("Drive Note 2", flipAlliance);
    follow("Score Note 2", flipAlliance);
    mSuperstructure.controlIntake(false, false);
    startTimeShooter = frc::Timer::GetFPGATimestamp().value();
    while (mSuperstructure.mShooter.getSpeed() < 4000 || frc::Timer::GetFPGATimestamp().value() - startTimeShooter > 3.0) {};
    // Run indexer
    mSuperstructure.mIndex.setVelocity(2000.0);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // Stop Shooter
    mSuperstructure.mIndex.setVelocity(0.0);
    mSuperstructure.mShooter.setSpeed(Shooter::STOP);
    


    // mSuperstructure.controlIntake(false, false);

    // mSuperstructure.controlIntake(true, false); 
    // follow("[M] Note 2", flipAlliance);
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    // mSuperstructure.controlIntake(false, false);
    // follow("[M] Score Note 2", flipAlliance);

    // mSuperstructure.mShooter.setSpeed(Shooter::HIGH);
    // // Wait until shooter reaches 4000 RPM or 5 seconds pass
    // startTimeShooter = frc::Timer::GetFPGATimestamp().value();
    // while (mSuperstructure.mShooter.getSpeed() < 4000 || startTimeShooter - frc::Timer::GetFPGATimestamp().value() > 5.0) {};
    // // Run indexer
    // mSuperstructure.mIndex.setVelocity(2000.0);
    // std::this_thread::sleep_for(std::chrono::seconds(2));
    // // Stop Shooter
    // mSuperstructure.mIndex.setVelocity(0.0);
    // mSuperstructure.mShooter.setSpeed(Shooter::STOP);

    // break;
    // switch (numPath)
    // {
    // // do nothing
    // case 0: 
    //     break; 

    // // straight past stage line
    // case 1:
    //     mSuperstructure.mShooter.setSpeed(Shooter::HIGH);
    //     // Wait until shooter reaches 4000 RPM or 5 seconds pass
    //     double startTimeShooter = frc::Timer::GetFPGATimestamp().value();
    //     while (mSuperstructure.mShooter.getSpeed() < 4000 || startTimeShooter - frc::Timer::GetFPGATimestamp().value() > 5.0) {};
    //     // Run indexer
    //     mSuperstructure.mIndex.setVelocity(2000.0);
    //     std::this_thread::sleep_for(std::chrono::seconds(2));
    //     // Stop Shooter
    //     mSuperstructure.mIndex.setVelocity(0.0);
    //     mSuperstructure.mShooter.setSpeed(Shooter::STOP);
    //     // follow("[R1] Straight", flipAlliance);
    //     break;

    // // R - 2 note auto  
    // // HC 5, holding HC 4 
    // // (when other teams want all inner notes)
    // case 2:
    //     mSuperstructure.preScoreSpeaker(); 
    //     mSuperstructure.scoreSpeaker(); 
    //     follow("[R] HC 5", flipAlliance);
    //     mSuperstructure.controlIntake(true, false);
    //     follow("[R] Score HC 5", flipAlliance);
    //     mSuperstructure.preScoreSpeaker(); 
    //     mSuperstructure.scoreSpeaker(); 

    //     follow("[R] HC 4", flipAlliance); 
    //     mSuperstructure.controlIntake(true, false);
    //     break;

    // // R - 3 note auto
    // // note 3, HC 5, holding HC 4
    // // (when other teams want inner notes 1 and 2)
    // case 3:
    //     mSuperstructure.preScoreSpeaker(); 
    //     mSuperstructure.scoreSpeaker(); 
    //     follow("[R] Note 3", flipAlliance);
    //     mSuperstructure.controlIntake(true, false);
    //     follow("[R] Score Note 3", flipAlliance);
    //     mSuperstructure.preScoreSpeaker(); 
    //     mSuperstructure.scoreSpeaker(); 

    //     follow("[R] HC 5", flipAlliance); 
    //     mSuperstructure.controlIntake(true, false);
    //     follow("[R] Score HC 5", flipAlliance); 
    //     mSuperstructure.preScoreSpeaker(); 
    //     mSuperstructure.scoreSpeaker(); 

    //     follow("[R] HC 4", flipAlliance); 
    //     mSuperstructure.controlIntake(true, false);
    //     break;

    // // R - 3 note auto 
    // // note 3, note 2, holding HC 5
    // case 4: 
    //     mSuperstructure.preScoreSpeaker(); 
    //     mSuperstructure.scoreSpeaker(); 
    //     follow("[R] Note 3", flipAlliance);
    //     mSuperstructure.controlIntake(true, false);
    //     follow("[R] Score Note 3", flipAlliance);
    //     mSuperstructure.preScoreSpeaker(); 
    //     mSuperstructure.scoreSpeaker(); 

    //     follow("[R] Note 2", flipAlliance); 
    //     mSuperstructure.controlIntake(true, false);
    //     follow("[R] Score Note 2", flipAlliance); 
    //     mSuperstructure.preScoreSpeaker(); 
    //     mSuperstructure.scoreSpeaker(); 

    //     follow("[R] HC 5", flipAlliance); 
    //     mSuperstructure.controlIntake(true, false);
    //     break; 

    // // M - 2 note auto 
    // // note 2, holding HC 2 
    // case 5:
    //     mSuperstructure.preScoreSpeaker(); 
    //     mSuperstructure.scoreSpeaker(); 
    //     follow("[M] Note 2", flipAlliance);
    //     mSuperstructure.controlIntake(true, false);
    //     follow("[M] Score Note 2", flipAlliance);
    //     mSuperstructure.preScoreSpeaker(); 
    //     mSuperstructure.scoreSpeaker(); 

    //     follow("[M] HC 2", flipAlliance);
    //     mSuperstructure.controlIntake(true, false);
    //     break;

    // // M - 3 note auto 
    // // note 2, note 3, holding HC 2
    // case 6: 
    //     mSuperstructure.preScoreSpeaker(); 
    //     mSuperstructure.scoreSpeaker(); 
    //     follow("[M] Note 2", flipAlliance);
    //     mSuperstructure.controlIntake(true, false);
    //     follow("[M] Score Note 2", flipAlliance);
    //     mSuperstructure.preScoreSpeaker(); 
    //     mSuperstructure.scoreSpeaker(); 

    //     follow("[M] Note 3", flipAlliance);
    //     mSuperstructure.controlIntake(true, false);
    //     follow("[M] Score Note 3", flipAlliance);
    //     mSuperstructure.preScoreSpeaker(); 
    //     mSuperstructure.scoreSpeaker(); 

    //     follow("[M] HC 2", flipAlliance); 
    //     mSuperstructure.controlIntake(true, false);
    //     break; 

    // // M - 4 note auto 
    // // note 2, note 3, note 1, holding HC 1
    // case 7: 
    //     mSuperstructure.preScoreSpeaker(); 
    //     mSuperstructure.scoreSpeaker(); 
    //     follow("[M] Note 2", flipAlliance);
    //     mSuperstructure.controlIntake(true, false);
    //     follow("[M] Score Note 2", flipAlliance);
    //     mSuperstructure.preScoreSpeaker(); 
    //     mSuperstructure.scoreSpeaker(); 

    //     follow("[M] Note 3", flipAlliance);
    //     mSuperstructure.controlIntake(true, false);
    //     follow("[M] Score Note 3", flipAlliance);
    //     mSuperstructure.preScoreSpeaker(); 
    //     mSuperstructure.scoreSpeaker(); 

    //     follow("[M] Note 1", flipAlliance);
    //     mSuperstructure.controlIntake(true, false);
    //     follow("[M] Score Note 1", flipAlliance);
    //     mSuperstructure.preScoreSpeaker(); 
    //     mSuperstructure.scoreSpeaker(); 

    //     follow("[M] HC 2", flipAlliance);
    //     mSuperstructure.controlIntake(true, false);
    //     break; 

    // // L - 2 note auto
    // // note 1, holding HC 1
    // case 8: 
    //     mSuperstructure.preScoreSpeaker(); 
    //     mSuperstructure.scoreSpeaker();
    //     follow("[L] Note 1", flipAlliance); 
    //     mSuperstructure.controlIntake(true, false);
    //     follow("[L] Score Note 1", flipAlliance);
    //     mSuperstructure.preScoreSpeaker(); 
    //     mSuperstructure.scoreSpeaker(); 
    //     follow ("[L] HC 1", flipAlliance); 
    //     mSuperstructure.controlIntake(true, false);
    //     break; 

    // // L - 3 note auto 
    // // note 1, note 2, holding HC 1
    // case 9: 
    //     mSuperstructure.preScoreSpeaker(); 
    //     mSuperstructure.scoreSpeaker();
    //     follow("[L] Note 1", flipAlliance); 
    //     mSuperstructure.controlIntake(true, false);
    //     follow("[L] Score Note 1", flipAlliance);
    //     mSuperstructure.preScoreSpeaker(); 
    //     mSuperstructure.scoreSpeaker(); 

    //     follow("[L] Note 2", flipAlliance); 
    //     mSuperstructure.controlIntake(true, false);
    //     follow("[L] Score Note 2", flipAlliance);
    //     mSuperstructure.preScoreSpeaker(); 
    //     mSuperstructure.scoreSpeaker(); 

    //     follow ("[L] HC 1", flipAlliance); 
    //     mSuperstructure.controlIntake(true, false);
    //     break; 
    // }
}