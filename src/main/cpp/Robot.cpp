// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit()
{
  mDrive.initModules();
  mGyro.init();
  mPowerModule.init(true);
  // mIntake.init();
}
void Robot::RobotPeriodic()
{
  mDrive.setDriveCurrentLimit(mPowerModule.updateDriveCurrentLimit());
  frc::SmartDashboard::PutNumber("Energy Usage", mPowerModule.mPDH.GetTotalEnergy());
  frc::SmartDashboard::PutNumber("Gyro", mGyro.getBoundedAngleCW().getDegrees());
  frc::SmartDashboard::PutNumber("Mag", mGyro.getMagnetometerCW().getDegrees());
  frc::SmartDashboard::PutNumber("GyroConnected?", mGyro.gyro.IsConnected());
}

void Robot::AutonomousInit()
{
  mGyro.init(); 
  mDrive.enableModules();
  mDrive.state = 'a';

  Trajectory mTraj = Trajectory(mDrive, mShooter, mLimelight);
  mTraj.followPath(1);

}
void Robot::AutonomousPeriodic()
{
}
void Robot::TeleopInit()
{
  mDrive.state = 't'; 
  mDrive.enableModules();
  mGyro.init();
  mHeadingController.setHeadingControllerState(SwerveHeadingController::SNAP);
  xStickLimiter.reset(0.0);
  yStickLimiter.reset(0.0);
}
void Robot::TeleopPeriodic()
{
  // Controller inputs
  bool boost = ctr.GetL2Axis() > 0;

  double leftX = ControlUtil::deadZonePower(ctr.GetLeftX(), ctrDeadzone, 1);
  double leftY = ControlUtil::deadZonePower(-ctr.GetLeftY(), ctrDeadzone, 1);

  leftX = ControlUtil::boostScaler(leftX, boost, boostPercent, ctrPercent);
  leftY = ControlUtil::boostScaler(leftY, boost, boostPercent, ctrPercent);

  leftX = xStickLimiter.calculate(leftX);
  leftY = yStickLimiter.calculate(leftY);

  double rightX = ControlUtil::deadZoneQuadratic(ctr.GetRightX(), ctrDeadzone);

  double rightTrigger = ctr.GetR2Axis();
  int dPad = ctr.GetPOV();
  bool rumbleController = false;
  // Intake::intakeState intakeMode = Intake::buttonsToState(ctr.GetL1Button(), ctr.GetR1Button());

  // Driver Information
  frc::SmartDashboard::PutNumber("leftX", leftX);
  frc::SmartDashboard::PutNumber("leftY", leftY);
  frc::SmartDashboard::PutBoolean("TargetFound?", mLimelight.isSpeakerTagDetected());
  if (!mGyro.gyro.IsConnected())
  {
    rumbleController = true;
  }

  // Teleop States
  bool driveTranslating = !(leftX == 0 && leftY == 0);
  bool driveTurning = !(rightX == 0);
  double rot = rightX * moduleMaxRot;
  bool preparingToShoot = rightTrigger > 0.2;

  // Decide drive modes
  if (dPad >= 0) // SNAP mode
  {
    // Snap condition
    mHeadingController.setHeadingControllerState(SwerveHeadingController::SNAP);
    mHeadingController.setSetpointPOV(dPad);
  }
  else if (preparingToShoot && !driveTurning) // ALIGN(scoring) mode
  {
    if (mLimelight.isSpeakerTagDetected())
    {
      Pose3d target = mLimelight.getTargetPoseRobotSpace();
      double angleOffset = Rotation2d::polarToCompass(atan2(target.y, target.x)) * 180 / M_PI;
      double zeroSetpoint = mGyro.getBoundedAngleCW().getDegrees() + angleOffset;
      mHeadingController.setHeadingControllerState(SwerveHeadingController::ALIGN);
      mHeadingController.setSetpoint(zeroSetpoint);
      // limit robot speed temporarily
      leftX = (leftX / ctrPercent) * ctrPercentAim;
      leftY = (leftY / ctrPercent) * ctrPercentAim;
    }
    else
    {
      rumbleController = true;
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
  if (ctr.GetCrossButtonReleased())
  {
    mGyro.init();
  }

  // Drive function
  mDrive.Drive(
      ChassisSpeeds(leftX * moduleMaxFPS, leftY * moduleMaxFPS, rot),
      mGyro.getBoundedAngleCCW(),
      mGyro.gyro.IsConnected());
  // mIntake.setState(intakeMode, true, 1.0);
  if (rumbleController)
  {
    ctr.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.5);
  }

  // Module Telemetry
  mDrive.displayDriveTelemetry();
  // PowerModule::updateCurrentLimits();
}

void Robot::DisabledInit()
{
  mDrive.stopModules();
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