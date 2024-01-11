// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit()
{
  mDrive.initAllMotors();
  mGyro.init();
}
void Robot::RobotPeriodic()
{
  frc::SmartDashboard::PutNumber("Gyro", mGyro.getBoundedAngleCW().getDegrees());
}

void Robot::AutonomousInit()
{
  mDrive.enableThreads();

}
void Robot::AutonomousPeriodic()
{
}
void Robot::TeleopInit()
{
  mDrive.enableThreads();
  mHeadingController.setHeadingControllerState(SwerveHeadingController::SNAP);
}
void Robot::TeleopPeriodic()
{
  // Controller inputs
  double leftX = ControlUtil::deadZoneQuadratic(ctr.GetLeftX() / 2, ctrDeadzone);
  double leftY = ControlUtil::deadZoneQuadratic(-ctr.GetLeftY() / 2, ctrDeadzone);
  double rightX = ControlUtil::deadZoneQuadratic(ctr.GetRightX(), ctrDeadzone);
  int dPad = ctr.GetPOV();

  // Teleop States
  bool drive_translating = !(leftX == 0 && leftY == 0);
  bool drive_turning = !(rightX == 0);
  double rot = rightX;

  if (dPad >= 0) {
    // Snap condition
    mHeadingController.setHeadingControllerState(SwerveHeadingController::SNAP);
    mHeadingController.setSetpointPOV(dPad);
  } else {
      mHeadingController.setHeadingControllerState(SwerveHeadingController::OFF);
  }

  rot = mHeadingController.getHeadingControllerState() == SwerveHeadingController::OFF
    ? rot : mHeadingController.calculate(mGyro.getBoundedAngleCW().getDegrees());
  // Clamp rot
  rot = std::clamp(rot, -1.0, 1.0);
  frc::SmartDashboard::PutNumber("rot" , rot);


  mDrive.Drive(
    rot, 
    ControlUtil::deadZoneQuadratic(ctr.GetLeftX() / 2, ctrDeadzone), 
    ControlUtil::deadZoneQuadratic(-ctr.GetLeftY() / 2, ctrDeadzone), 
    mGyro.getBoundedAngleCCW().getRadians());
  
  mDrive.displayDriveTelemetry();

  // Gyro Resets
  if (ctr.GetCrossButtonReleased()) 
  {
    mGyro.init();
  }

}

void Robot::DisabledInit()
{
  mDrive.stopAllMotors();
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
