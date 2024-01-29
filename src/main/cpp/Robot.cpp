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
}

void Robot::AutonomousInit()
{
  mDrive.enableModules();
  mDrive.autoMove(M_PI / 4, 5.0);
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
  double leftRawX = ControlUtil::deadZonePower(ctr.GetLeftX(), ctrDeadzone, 1) * ctrPercent;
  double leftRawY = ControlUtil::deadZonePower(-ctr.GetLeftY(), ctrDeadzone, 1) * ctrPercent;
  double rightX = ControlUtil::deadZoneQuadratic(ctr.GetRightX(), ctrDeadzone);

  leftX = ControlUtil::limitPositiveAcceleration(leftX, leftRawX, 0.1, 1.0);
  leftY = ControlUtil::limitPositiveAcceleration(leftY, leftRawY, 0.1, 1.0);
  

  int dPad = ctr.GetPOV();

  // Teleop States
  bool drive_translating = !(leftX == 0 && leftY == 0);
  bool drive_turning = !(rightX == 0);
  double rot = rightX;

  // Decide drive modes
  if (dPad >= 0)
  {
    // Snap condition
    mHeadingController.setHeadingControllerState(SwerveHeadingController::SNAP);
    mHeadingController.setSetpointPOV(dPad);
  }
  else
  {
    mHeadingController.setHeadingControllerState(SwerveHeadingController::OFF);
  }
  rot = mHeadingController.getHeadingControllerState() == SwerveHeadingController::OFF
            ? rot
            : mHeadingController.calculate(mGyro.getBoundedAngleCW().getDegrees());

  frc::SmartDashboard::PutNumber("rot", rot);

  // Gyro Resets
  if (ctr.GetCrossButtonReleased())
  {
    mGyro.init();
  }

  // Drive function
  mDrive.Drive(
      rot,
      leftX,
      leftY,
      mGyro.getBoundedAngleCCW().getRadians());

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
