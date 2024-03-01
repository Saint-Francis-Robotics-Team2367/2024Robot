// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include<frc/DriverStation.h>
#include <string>

void Robot::RobotInit()
{
  mElevator.init();
  mDrive.initModules();
  mGyro.init();
  // mPowerModule.init(true);
  mSuperstructure.init();

  mChooser.SetDefaultOption("0", kAutoDefault);
  mChooser.AddOption("1", kAutoCustom1);
  mChooser.AddOption("2", kAutoCustom2);
  mChooser.AddOption("3", kAutoCustom3);
  mChooser.AddOption("4", kAutoCustom4);
  frc::SmartDashboard::PutData("Auto Paths", &mChooser);
}
void Robot::RobotPeriodic()
{
  // mDrive.setDriveCurrentLimit(mPowerModule.updateDriveCurrentLimit());
  // frc::SmartDashboard::PutNumber("Energy Usage", mPowerModule.mPDH.GetTotalEnergy());
  frc::SmartDashboard::PutNumber("Shooter Angle", mSuperstructure.mArm.getShooterAngle().getDegrees());
  frc::SmartDashboard::PutNumber("Gyro", mGyro.getBoundedAngleCW().getDegrees());
  frc::SmartDashboard::PutNumber("IntakeCurrent", mSuperstructure.mIntake.intakeMotor.GetOutputCurrent());
}

void Robot::AutonomousInit()
{
  mGyro.init();
  mDrive.enableModules();

  mDrive.state = DriveState::Auto;
  mSuperstructure.enable();
  selectedAuto = mChooser.GetSelected();
  frc::SmartDashboard::PutString("auto", selectedAuto);

  Trajectory mTraj = Trajectory(mDrive, mSuperstructure);
  mTraj.followPath(1, false);
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
  mHeadingController.setHeadingControllerState(SwerveHeadingController::SNAP);
  xStickLimiter.reset(0.0);
  yStickLimiter.reset(0.0);
}
void Robot::TeleopPeriodic()
{
  auto startTime = frc::Timer::GetFPGATimestamp();
  // Controller inputs
  bool boost = ctr.GetL2Axis() > 0;

  double leftX = ControlUtil::deadZonePower(ctr.GetLeftX(), ctrDeadzone, 1);
  double leftY = ControlUtil::deadZonePower(-ctr.GetLeftY(), ctrDeadzone, 1);

  leftX = ControlUtil::boostScaler(leftX, boost, boostPercent, ctrPercent);
  leftY = ControlUtil::boostScaler(leftY, boost, boostPercent, ctrPercent);

  leftX = xStickLimiter.calculate(leftX);
  leftY = yStickLimiter.calculate(leftY);

  double rightX = ControlUtil::deadZoneQuadratic(ctr.GetRightX(), ctrDeadzone);

  int dPad = ctr.GetPOV();
  bool rumbleController = false;

  // Driver Information
  frc::SmartDashboard::PutNumber("leftX", leftX);
  frc::SmartDashboard::PutNumber("leftY", leftY);
  frc::SmartDashboard::PutBoolean("TargetFound?", mLimelight.isSpeakerTagDetected());

  // Teleop States
  bool driveTranslating = !(leftX == 0 && leftY == 0);
  bool driveTurning = !(rightX == 0);
  double rot = rightX * moduleMaxRot;
  bool preScoringSpeaker = ctr.GetR2Axis() > 0.2;
  bool intakeIn = ctr.GetR1Button();
  bool intakeClear = ctr.GetL1Button();
  bool shootNote = ctr.GetTriangleButton();
  bool loadNote = ctr.GetCrossButton();
  // if (ctr.GetCreateButtonPressed()) {
  //   liftElev = !liftElev;
  // }
  if (ctr.GetTriangleButtonReleased())
  {
    scoreAmp = !scoreAmp;
  }

  // Decide drive modes
  if (snapRobotToGoal.update(dPad >= 0 && !driveTurning, 5.0, driveTurning)) // SNAP mode
  {
    // Snap condition
    mHeadingController.setHeadingControllerState(SwerveHeadingController::SNAP);
    mHeadingController.setFieldSetpoint(dPad);
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
      // limit robot speed temporarily
      leftX = (leftX / ctrPercent) * ctrPercentAim;
      leftY = (leftY / ctrPercent) * ctrPercentAim;
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
  if (ctr.GetCircleButtonReleased())
  {
    mGyro.init();
  }

  // Drive function
  mDrive.Drive(
      ChassisSpeeds(leftX * moduleMaxFPS, leftY * moduleMaxFPS, -rot),
      mGyro.getBoundedAngleCCW(),
      mGyro.gyro.IsConnected());
  
  // if (liftElev) {
  //   mElevator.setState(Elevator::HIGH);
  // } else {
  //   mElevator.setState(Elevator::STOW);
  // }

  // Superstructure function
  

  if (loadNote && !scoreAmp) // Load note into shooter
  {
    if (ctr.GetCrossButtonPressed()) {
      mSuperstructure.loadNote();
    }
    
    // mSuperstructure.mIndex.setVelocity(3000);
    
    
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
    mSuperstructure.preScoreSpeaker();
    if (ctr.GetSquareButton()) // Load note into spinning shooter
    {
      mSuperstructure.scoreSpeaker();
    }
  }
  else
  {
    mSuperstructure.stow();
    mSuperstructure.controlIntake(intakeIn, intakeClear);
  }

  // Module Telemetry
  mDrive.displayDriveTelemetry();
  mSuperstructure.updateTelemetry();
  // PowerModule::updateCurrentLimits();
  static units::time::second_t max_loop{0};
  auto curr_loop = frc::Timer::GetFPGATimestamp() - startTime;
  if( curr_loop > max_loop)
    max_loop = curr_loop;
  frc::SmartDashboard::PutNumber("CurrLoop", curr_loop.value());
  frc::SmartDashboard::PutNumber("maxLoop", max_loop.value()); 

  mElevator.motorLeft.Set(ctrOperator.GetLeftY());
  mElevator.motorRight.Set(ctrOperator.GetRightY());  

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