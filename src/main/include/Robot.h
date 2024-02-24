// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/PS5Controller.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SendableChooser.h>

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
#include "Trajectory.h"
#include "Superstructure.h"
#include "util/TimeDelayButton.h"

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
  NavX mGyro = NavX();
  SwerveDrive mDrive = SwerveDrive(mGyro);
  Limelight mLimelight;
  Superstructure mSuperstructure;

  // Teleop Controls
  float ctrPercent = 0.5;
  float boostPercent = 0.9;
  double ctrPercentAim = 0.3;
  TimeDelayButton snapRobotToGoal;

  // Controllers
  SwerveHeadingController mHeadingController = SwerveHeadingController(-1.0, 1.0);
  SlewRateLimiter xStickLimiter = SlewRateLimiter(ctrSlewRate);
  SlewRateLimiter yStickLimiter = SlewRateLimiter(ctrSlewRate);
  PowerModule mPowerModule;

  // Auto Selectors
  frc::SendableChooser<std::string> mChooser;
  const std::string kAutoDefault = "Default: nothing";
  const std::string kAutoCustom1 = "Straight";
  const std::string kAutoCustom2 = "Straight, shoot";
  const std::string kAutoCustom3 = "2 straights, shoot";
  const std::string kAutoCustom4 = "Curve";
  std::string selectedAuto;
};
