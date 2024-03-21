// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <string>
#include "LimelightAlign.cpp"

void Robot::RobotInit()
{
  mElevator.init();
  mDrive.initModules();
  mGyro.init();
  mSuperstructure.init();

  mChooser.SetDefaultOption("Middle Three Piece", Trajectory::MIDDLE_THREE_PIECE);
  mChooser.AddOption("Delayed Shoot, No Move", Trajectory::DELAYED_SHOOT_NO_MOVE);
  mChooser.AddOption("Amp 3 piece", Trajectory::AMP_THREE_PIECE);
  mChooser.AddOption("Source 3 piece", Trajectory::SOURCE_THREE_PIECE);
  frc::SmartDashboard::PutData("Auto Paths", &mChooser);
}
void Robot::RobotPeriodic()
{
  frc::SmartDashboard::PutNumber("Shooter Angle", mSuperstructure.mArm.getShooterAngle().getDegrees());
  frc::SmartDashboard::PutNumber("Gyro", mGyro.getBoundedAngleCW().getDegrees());
  frc::SmartDashboard::PutNumber("IntakeCurrent", mSuperstructure.mIntake.intakeMotor.GetOutputCurrent());
  frc::SmartDashboard::PutNumber("Index vel", mSuperstructure.mIndex.indexEncoder.GetVelocity());
  frc::SmartDashboard::PutNumber("Index applied", mSuperstructure.mIndex.indexMotor.GetAppliedOutput());
  
  Pose3d target = mLimelight.getTargetPoseRobotSpace();
  frc::SmartDashboard::PutNumber("LimelightX", target.x * 39.37);
  frc::SmartDashboard::PutNumber("LimelightY", target.y * 39.37);
  frc::SmartDashboard::PutNumber("TestShooterAngle", mSuperstructure.mArm.findBetterLaunchAngle(target.x * 39.37, target.y * 39.37, 51.875) * 180 / PI);
}

void Robot::AutonomousInit()
{
  mDrive.state = DriveState::Auto;

  mGyro.init();
  mDrive.enableModules();
  mSuperstructure.enable();

  selectedAuto = mChooser.GetSelected();
  Trajectory mTraj = Trajectory(mDrive, mSuperstructure, mGyro, mLimelight);
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
  frc::SmartDashboard::PutBoolean("TargetFound?", mLimelight.isSpeakerTagDetected());
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
  bool leftTrigger = ctr.GetL2Axis() > 0.5;
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
  else if (preScoringSpeaker && !driveTurning) // ALIGN(scoring) mode
  {
    if (mLimelight.isSpeakerTagDetected())
    {
      Pose3d target = mLimelight.getTargetPoseRobotSpace();
      double angleOffset = Rotation2d::polarToCompass(atan2(target.y, target.x)) * 180 / PI;
      double zeroSetpoint = mGyro.getBoundedAngleCW().getDegrees() + angleOffset;
      mHeadingController.setHeadingControllerState(SwerveHeadingController::ALIGN);
      mHeadingController.setSetpoint(zeroSetpoint);
    }
  }
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
  else if (preScoringSpeaker) // Spin Shooter
  {
    Pose3d target = mLimelight.getTargetPoseRobotSpace();
    mSuperstructure.preScoreSpeaker(target);
    if ((ctr.GetSquareButton() && mSuperstructure.mShooter.getSpeed() > 4000) || overrideShooter) // Load note into spinning shooter
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

  //Limelight align test-code for AMP
  while(leftTrigger){
    llAutoRotation();
    mDrive.autoMove(0, horizontalMovementAmp(mLimelight));
    if (autoShoot(mLimelight)){
      mSuperstructure.scoreAmp();
      mSuperstructure.unloadShooter();
    }
  }
}
void Robot::DisabledInit()
{
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