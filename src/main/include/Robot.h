// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/PS5Controller.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include "util/ShuffleUI.h"
#include <thread>

#include "util/ControlUtil.h"
#include "sensors/NavX.h"
#include "swerve/SwerveHeadingController.h"
#include "util/TimeDelayedBool.h"
#include <frc/Joystick.h>
#include "sensors/Limelight.h"
#include "util/SlewRateLimiter.h"
#include <frc/GenericHID.h>
#include "control/PowerModule.h"

#include "SwerveDrive.h"
// #include "Intake.h"

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  void SimulationInit() override;
  void SimulationPeriodic() override;

  // Modules/Devices
  frc::PS5Controller ctr = frc::PS5Controller(0);
  SwerveDrive mDrive = SwerveDrive();
  NavX mGyro = NavX();
  Limelight mLimelight;
  // Intake mIntake;
  // frc::Joystick ctr = frc::Joystick(0);

  // Teleop Controls
  float ctrPercent = 0.5;
  float boostPercent = 0.8;
  double ctrPercentAim = 0.3;

  // Controllers
  SwerveHeadingController mHeadingController = SwerveHeadingController(-1.0, 1.0);
  SlewRateLimiter xStickLimiter = SlewRateLimiter(ctrSlewRate);
  SlewRateLimiter yStickLimiter = SlewRateLimiter(ctrSlewRate);
  // PowerModule mPowerModule;
  
};
