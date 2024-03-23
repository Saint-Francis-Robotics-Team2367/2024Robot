#include "Trajectory.h"

#include <frc/controller/HolonomicDriveController.h>
#include <frc/kinematics/ChassisSpeeds.h>

// controller used to track trajectories + correct minor disturbances
static frc::HolonomicDriveController controller{
    frc::PIDController{revkP, 0, 0},
    frc::PIDController{revkP, 0, 0},
    frc::ProfiledPIDController<units::radian>{
        0.4, 0, 0,
        frc::TrapezoidProfile<units::radian>::Constraints{
            units::radians_per_second_t(5.0),
            units::radians_per_second_squared_t(100)}}};

/**
 * Drives robot to the next state on trajectory
 * Odometry must be in meters
 */
void Trajectory::driveToState(PathPlannerTrajectory::State const &state)
{
    // Calculate new chassis speeds given robot position and next desired state in trajectory
    frc::ChassisSpeeds const correction = controller.Calculate(mDrive.getOdometryPose(), frc::Pose2d{state.position, state.heading}, state.velocity, state.targetHolonomicRotation);

    // Calculate x, y speeds from MPS
    double vx_feet = correction.vx.value() * 3.281;
    double vy_feet = correction.vy.value() * 3.281;

    // Clamp rot speed to 2.0 since that is the max rot we allow
    double rot = std::clamp(correction.omega.value(), -moduleMaxRot, moduleMaxRot);

    frc::SmartDashboard::PutNumber("autoVY", vy_feet);
    frc::SmartDashboard::PutNumber("autoVX", vx_feet);
    frc::SmartDashboard::PutNumber("autoRot", rot);

    mDrive.Drive(ChassisSpeeds{-vy_feet, vx_feet, rot}, mGyro.getBoundedAngleCCW(), true, true);
}

/**
 * Follows pathplanner trajectory
 */
void Trajectory::follow(std::string const &traj_dir_file_path, bool flipAlliance, bool intake, bool first, float startAngle)
{
    mDrive.enableModules();
    auto path = PathPlannerPath::fromPathFile(traj_dir_file_path);

    // switches path to red alliance (mirrors it)
    if (flipAlliance)
    {
        path = path->flipPath();
    }

    PathPlannerTrajectory traj = PathPlannerTrajectory(path, frc::ChassisSpeeds(), 0_rad);

    if (first)
    {
        auto const initialState = traj.getInitialState();
        auto const initialPose = initialState.position;

        // set second param to initial holonomic rotation
        mDrive.resetOdometry(initialPose, units::angle::degree_t(startAngle));
    }

    frc::Timer trajTimer;
    trajTimer.Start();

    while ((mDrive.state == DriveState::Auto) && (trajTimer.Get() <= traj.getTotalTime()))
    {
        if (intake)
        {
            mSuperstructure.controlIntake(true, false);
        }

        auto currentTime = trajTimer.Get();
        auto sample = traj.sample(currentTime);

        driveToState(sample);
        mDrive.updateOdometry();

        frc::SmartDashboard::PutNumber("curr pose x meters", mDrive.getOdometryPose().Translation().X().value());
        frc::SmartDashboard::PutNumber("curr pose y meters", mDrive.getOdometryPose().Translation().Y().value());

        using namespace std::chrono_literals;

        // refresh rate of holonomic drive controller's PID controllers (edit if needed)
        double delayStart = frc::Timer::GetFPGATimestamp().value();
        while (mDrive.state == DriveState::Auto && frc::Timer::GetFPGATimestamp().value() - delayStart < 0.02) {
        };
    }
    mDrive.stopModules();
}

/**
 * Uses limelight to calculate error accumulated in path
 */
void Trajectory::driveError()
{
    // double x = mLimelight.getXYCoords()[0];
    // double y = mLimelight.getXYCoords()[1];

    // double currX = mDrive.getOdometryPose().X().value();
    // double currY = mDrive.getOdometryPose().Y().value();
}

/**
 * Calls sequences of follow functions for set paths
 * Path naming convention: "[Right Middle Left] Action"
 */
void Trajectory::followPath(Trajectory::autos autoTrajectory, bool flipAlliance)
{
    switch (autoTrajectory)
    {
    case DO_NOTHING:
        break;

    case DELAYED_SHOOT_NO_MOVE:
        waitToShoot(1);
        break;
    case MIDDLE_TWO_PIECE:
        waitToShoot(1);

        follow("[M] Note 2", flipAlliance, true, true, 0.0);
        follow("[M] Score Note 2", flipAlliance, true, false);
        mSuperstructure.controlIntake(false, false);
        
        waitToShoot(1);

        follow("[M] Note 1", flipAlliance, true, false);
        follow("[M] Score Note 1", flipAlliance, true, false);
        mSuperstructure.controlIntake(false, false);

        waitToShoot(1);
        
        break;

    case MIDDLE_THREE_PIECE:
        waitToShoot(1);

        follow("[M] Note 2", flipAlliance, true, true, 0.0);
        follow("[M] Score Note 2", flipAlliance, true, false);
        mSuperstructure.controlIntake(false, false);
        
        waitToShoot(1);

        follow("[M] Note 1", flipAlliance, true, false);
        follow("[M] Score Note 1", flipAlliance, true, false);
        mSuperstructure.controlIntake(false, false);

        waitToShoot(1);

        follow("[M] Note 3", flipAlliance, true, false);
        follow("[M] Score Note 3", flipAlliance, true, false);
        mSuperstructure.controlIntake(false, false);

        waitToShoot(1);

        break;

    case AMP_THREE_PIECE:
        follow("[Amp] Drive Note 1", flipAlliance, false, true, 60.0);
        break;

    case SOURCE_THREE_PIECE:
        follow("[R] HC 5", flipAlliance, true, true, -60.0);
        follow("[R] Score HC 5", flipAlliance, true, false);
        mSuperstructure.controlIntake(false, false);
        waitToShoot(1);

        follow("[R] HC 4", flipAlliance, true, false);
        follow("[R] Score HC 4", flipAlliance, true, false);
        mSuperstructure.controlIntake(false, false);

        waitToShoot(1);

        follow("[R] Note 3", flipAlliance, true, false);
        follow("[R] Score Note 3", flipAlliance, true, false);
        mSuperstructure.controlIntake(false, false);
        waitToShoot(1);
        break;

    case STEAL_HC_NOTES:
        follow("[R] HC 5", flipAlliance, true, true);
        mSuperstructure.controlIntake(false, false);
        waitToShoot(0.2); //Shoot note
        follow("Rotate HC 4 Part 1", flipAlliance, true, false);
        follow("Rotate Back 1", flipAlliance, false, false);
        mSuperstructure.controlIntake(false, false);
        waitToShoot(0.2); //Shoot note
        follow("Rotate HC 3 Part 1", flipAlliance, true, false);
        follow("Rotate Back 2", flipAlliance, false, false);
        mSuperstructure.controlIntake(false, false);
        waitToShoot(0.2); //Shoot note
        follow("Rotate HC 2 Part 1", flipAlliance, true, false);
        follow("Rotate Back 3", flipAlliance, false, false);
        mSuperstructure.controlIntake(false, false);
        waitToShoot(0.2); //Shoot note
        follow("Rotate HC 1 Part 1", flipAlliance, true, false);
        follow("Rotate Back 4", flipAlliance, false, false);
        mSuperstructure.controlIntake(false, false);
        waitToShoot(0.2); //Shoot note
        break;
    case AMP_SHOOT_AND_PARK:
        waitToShoot(1);
        std::this_thread::sleep_for(std::chrono::seconds(6));
        follow("[Amp] Park", flipAlliance, false, true, 60.0);
        break;
    case SOURCE_SHOOT_AND_PARK:
        waitToShoot(1);
        std::this_thread::sleep_for(std::chrono::seconds(6));
        follow("[Source] Park", flipAlliance, false, true, -60.0);
        break;
    case SOURCE_STEAL_TO_LEFT:
        follow("[Source] Steal to Left", false, false, true, 0.0);
        break;
    case SOURCE_STEAL_TO_RIGHT:
        follow("[Source] Steal to Right", false, false, true, 0.0);
        break;
    }
}

void Trajectory::waitToShoot(int delaySeconds)
{
    mSuperstructure.mShooter.setSpeed(Shooter::HIGH);
    // Wait until shooter reaches 4000 RPM or 3 seconds pass
    double startTimeShooter = frc::Timer::GetFPGATimestamp().value();
    while (mDrive.state == DriveState::Auto && (mSuperstructure.mShooter.getSpeed() < 4000 || frc::Timer::GetFPGATimestamp().value() - startTimeShooter < 3.0))
    {
    };
    // Run indexer
    mSuperstructure.mIndex.setVelocity(5700.0);
    double startTimeShoot = frc::Timer::GetFPGATimestamp().value();
    while (mDrive.state == DriveState::Auto && frc::Timer::GetFPGATimestamp().value() - startTimeShoot < delaySeconds) {

    };
    // Stop Shooter
    mSuperstructure.mIndex.setVelocity(0.0);
}