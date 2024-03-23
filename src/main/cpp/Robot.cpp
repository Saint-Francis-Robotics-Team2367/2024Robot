// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <string>

void Robot::RobotInit()
{
  mElevator.init();
  mDrive.initModules();
  mGyro.init();
  mSuperstructure.init();

  mChooser.SetDefaultOption("Middle Three Piece", Trajectory::MIDDLE_THREE_PIECE);
  mChooser.AddOption("Middle Two Piece", Trajectory::MIDDLE_TWO_PIECE);
  mChooser.AddOption("Delayed Shoot, No Move", Trajectory::DELAYED_SHOOT_NO_MOVE);
  mChooser.AddOption("Source 3 piece", Trajectory::SOURCE_THREE_PIECE);
  mChooser.AddOption("Source Steal Left", Trajectory::SOURCE_STEAL_TO_LEFT);
  mChooser.AddOption("Source Steal Right", Trajectory::SOURCE_STEAL_TO_RIGHT);
  frc::SmartDashboard::PutData("Auto Paths", &mChooser);
}
void Robot::RobotPeriodic()
{
  frc::SmartDashboard::PutNumber("Shooter Angle", mSuperstructure.mArm.getShooterAngle().getDegrees());
  frc::SmartDashboard::PutNumber("Gyro", mGyro.getBoundedAngleCW().getDegrees());
  
  // Pose3d target = mLimelight.getTargetPoseRobotSpace();
  // frc::SmartDashboard::PutNumber("targetX", target.x * 39.37);
  // frc::SmartDashboard::PutNumber("targetY", target.y * 39.37);

}

void Robot::AutonomousInit()
{
  mDrive.state = DriveState::Auto;

  mGyro.init();
  mDrive.enableModules();
  mSuperstructure.enable();

  selectedAuto = mChooser.GetSelected();
  Trajectory mTraj = Trajectory(mDrive, mSuperstructure, mGyro, mLimelight);
  // if (frc::DriverStation::IsDSAttached()) {
  //   mTraj.isRed = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed;
  // }
  // if (mLimelight.targetDetected()) {
  //   mTraj.startPose = mLimelight.getRobotPoseFieldSpace();
  //   mTraj.receivedPose = true;
  // } else {
  //   mTraj.receivedPose = false;
  // }
  
  
  mTraj.followPath(selectedAuto, false);
}
void Robot::AutonomousPeriodic()
{
}
void Robot::TeleopInit()
{
  mDrive.state = DriveState::Teleop;
  mDrive.enableModules();
  mSuperstructure.enable();
  mGyro.init();
  mDrive.resetOdometry(frc::Translation2d(0_m, 0_m), frc::Rotation2d(0_rad));
  mSuperstructure.mShooter.setSpeed(Shooter::STOP);

  mHeadingController.setHeadingControllerState(SwerveHeadingController::OFF);
  xStickLimiter.reset(0.0);
  yStickLimiter.reset(0.0);
}
void Robot::TeleopPeriodic()
{
  auto startTime = frc::Timer::GetFPGATimestamp();
  // Controller inputs
  double leftX = ControlUtil::deadZonePower(ctr.GetLeftX(), ctrDeadzone, 1);
  double leftY = ControlUtil::deadZonePower(-ctr.GetLeftY(), ctrDeadzone, 1);

  leftX = xStickLimiter.calculate(leftX); 
  leftY = yStickLimiter.calculate(leftY);

  double rightX = ControlUtil::deadZoneQuadratic(ctr.GetRightX(), ctrDeadzone);

  int dPad = ctrOperator.GetPOV();
  bool rumbleController = false;

  // Driver Information
  //frc::SmartDashboard::PutBoolean("TargetFound?", mLimelight.isSpeakerTagDetected());
  frc::SmartDashboard::PutBoolean("Note?", mSuperstructure.mIndex.isNoteDetected());
  frc::SmartDashboard::PutNumber("Prox?", mSuperstructure.mIndex.getSensorProximity());

  // Teleop States
  bool driveTranslating = !(leftX == 0 && leftY == 0);
  bool driveTurning = !(rightX == 0);
  double rot = rightX * moduleMaxRot * 2;
  bool preScoringSpeaker = ctr.GetR2Axis() > 0.2;
  bool intakeIn = ctr.GetR1Button();
  bool intakeClear = ctr.GetL1Button();
  bool loadNote = ctr.GetCrossButton();
  bool reverseNote = ctr.GetCircleButton();
  bool overrideShooter = ctrOperator.GetSquareButton();
  if (ctr.GetTriangleButtonReleased())
  {
    scoreAmp = !scoreAmp;
  }

  // Decide drive modes
  if (snapRobotToGoal.update(dPad >= 0 && !driveTurning, 5.0, driveTurning)) // SNAP mode
  {
    mHeadingController.setHeadingControllerState(SwerveHeadingController::SNAP);
    mHeadingController.setSetpointPOV(dPad);
  }
  // else if (preScoringSpeaker && !driveTurning) // ALIGN(scoring) mode
  // {
  //   // if (mLimelight.isSpeakerTagDetected())
  //   // {
  //   //   Pose3d target = mLimelight.getTargetPoseRobotSpace();
  //   //   double angleOffset = Rotation2d::polarToCompass(atan2(target.y, target.x)) * 180 / PI;
  //   //   double zeroSetpoint = mGyro.getBoundedAngleCW().getDegrees() + angleOffset;
  //   //   mHeadingController.setHeadingControllerState(SwerveHeadingController::ALIGN);
  //   //   mHeadingController.setSetpoint(zeroSetpoint);
  //   // }
  // }
  else // Normal driving mode
  {
    mHeadingController.setHeadingControllerState(SwerveHeadingController::OFF);
  }

  // Output heading controller if used
  rot = mHeadingController.getHeadingControllerState() == SwerveHeadingController::OFF
            ? rot
            : mHeadingController.calculate(mGyro.getBoundedAngleCW().getDegrees());

  // Gyro Resets
  if (ctrOperator.GetCrossButtonReleased())
  {
    mGyro.init();
  }

  // Drive function
  mDrive.Drive(
      ChassisSpeeds(leftX * moduleMaxFPS, leftY * moduleMaxFPS, -rot),
      mGyro.getBoundedAngleCCW(),
      mGyro.gyro.IsConnected(),
      cleanDriveAccum);
  mDrive.updateOdometry();
  frc::SmartDashboard::PutNumber("driveX", mDrive.getOdometryPose().X().value());
  frc::SmartDashboard::PutNumber("driveY", mDrive.getOdometryPose().Y().value());

  // Superstructure function

  if (loadNote && !scoreAmp) // Load note into shooter
  {
    if (ctr.GetCrossButtonPressed())
    {
      mSuperstructure.loadNote();
    }
  }
  else if (reverseNote && !scoreAmp)
  {
    if (ctr.GetCircleButtonPressed())
    {
      mSuperstructure.pushNoteBack();
    }
  }
  else if (scoreAmp) // Lift Arm
  {
    mSuperstructure.scoreAmp();
    if (ctr.GetSquareButton()) // Unload shooter into amp
    {
      mSuperstructure.unloadShooter();
    }
    else
    {
      mSuperstructure.mShooter.setSpeed(0.0);
    }
  }
  else if (preScoringSpeaker) // Spin Shooter print(hello world)
  {
    mSuperstructure.preScoreSpeaker();
    if ((ctr.GetSquareButton() && mSuperstructure.mShooter.getSpeed() > 5000) || overrideShooter) // Load note into spinning shooter
    {
      mSuperstructure.scoreSpeaker();
    }
  }
  else
  {
    if (ctr.GetR1ButtonReleased()) // lock shooter after intake
    {
      mSuperstructure.mShooter.setDistance(0.0);
    }
    if (ctr.GetR1ButtonPressed()) // unlock shooter before intake
    {
      mSuperstructure.mShooter.setSpeed(Shooter::STOP);
    }
    mSuperstructure.controlIntake(intakeIn, intakeClear);
    mSuperstructure.stow();
  }

  // Module Telemetry
  mDrive.displayDriveTelemetry();
  mSuperstructure.updateTelemetry();
  static units::time::second_t max_loop{0};
  auto curr_loop = frc::Timer::GetFPGATimestamp() - startTime;
  if (curr_loop > max_loop)
    max_loop = curr_loop;
  frc::SmartDashboard::PutNumber("CurrLoop", curr_loop.value());
  frc::SmartDashboard::PutNumber("maxLoop", max_loop.value());
  double elevatorSetpoint = ctrOperator.GetLeftY();
  if (ctrOperator.GetTriangleButton())
  {
    mElevator.motorLeft.Set(elevatorSetpoint);
    mElevator.motorRight.Set(elevatorSetpoint);
  }
  else
  {
    mElevator.motorLeft.StopMotor();
    mElevator.motorRight.StopMotor();
  }
}

void Robot::DisabledInit()
{
  mDrive.state = DriveState::Disabled;
  mDrive.stopModules();
  mSuperstructure.disable();
}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif