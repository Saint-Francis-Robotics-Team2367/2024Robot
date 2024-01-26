// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit()
{
  mDrive.initModules();
  mGyro.init();
}
void Robot::RobotPeriodic()
{
  frc::SmartDashboard::PutNumber("Gyro", mGyro.getBoundedAngleCW().getDegrees());
  frc::SmartDashboard::PutNumber("Mag", mGyro.getMagnetometerCW().getDegrees());
  frc::SmartDashboard::PutNumber("GyroConnected?", mGyro.gyro.IsConnected());
}

void Robot::AutonomousInit()
{
  mDrive.enableModules();
}
void Robot::AutonomousPeriodic()
{
}
void Robot::TeleopInit()
{
  mDrive.enableModules();
  mGyro.init();
  mHeadingController.setHeadingControllerState(SwerveHeadingController::SNAP);
}
void Robot::TeleopPeriodic()
{
  // Controller inputs
  double leftX = ControlUtil::deadZonePower(ctr.GetLeftX(), ctrDeadzone, 1) * ctrPercent;
  double leftY = ControlUtil::deadZonePower(-ctr.GetLeftY(), ctrDeadzone, 1) * ctrPercent;
  double rightX = ControlUtil::deadZoneQuadratic(ctr.GetRightX(), ctrDeadzone);
  double leftTrigger = ctr.GetL2Axis();
  int dPad = ctr.GetPOV();

  // Driver Information
  frc::SmartDashboard::PutNumber("leftX", leftX);
  frc::SmartDashboard::PutNumber("leftY", leftY);
  frc::SmartDashboard::PutBoolean("TargetFound?", mLimelight.isSpeakerTagDetected());
  if (!mGyro.gyro.IsConnected()) {
    ctr.SetRumble(frc::GenericHID::RumbleType::kRightRumble, 0.5);
  }
  ctr.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.5);

  // Teleop States
  bool driveTranslating = !(leftX == 0 && leftY == 0);
  bool driveTurning = !(rightX == 0);
  double rot = rightX;
  bool preparingToShoot = leftTrigger > 0.2;

  // Decide drive modes
  if (dPad >= 0)
  {
    // Snap condition
    mHeadingController.setHeadingControllerState(SwerveHeadingController::SNAP);
    mHeadingController.setSetpointPOV(dPad);
  }
  else if (preparingToShoot && !driveTurning) 
  {
    if (mLimelight.isSpeakerTagDetected()) {
      Pose3d target = mLimelight.getTargetPoseRobotSpace();
      double angleOffset = Rotation2d::polarToCompass(atan2(target.y, target.x)) * 180 / M_PI;
      double zeroSetpoint = mGyro.getBoundedAngleCW().getDegrees() + angleOffset;
      mHeadingController.setHeadingControllerState(SwerveHeadingController::ALIGN);
      mHeadingController.setSetpoint(zeroSetpoint);
      // limit robot speed temporarily
      leftX = (leftX / ctrPercent) * 0.3;
      leftY = (leftY / ctrPercent) * 0.3;

    } else {
      ctr.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.5);
    }
  }
  else
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
    ChassisSpeeds(leftX * moduleMaxFPS, leftY * moduleMaxFPS, rot * moduleMaxRot), 
    mGyro.getBoundedAngleCCW(),
    mGyro.gyro.IsConnected()
  );

  // Module Telemetry
  mDrive.displayDriveTelemetry();
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
