#pragma once

#include <rev/CANSparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DutyCycleEncoder.h>
#include "geometry/Rotation2d.h"
#include <frc/controller/PIDController.h>
#include <rev/SparkPIDController.h>
#include <rev/SparkRelativeEncoder.h>
#include <thread>

constexpr int leftFrontMotorID = 9;
constexpr int leftBackMotorID = 11;
constexpr int rightFrontMotorID = 10;
constexpr int rightBackMotorID = 12;

constexpr int tiltControllerP = 0.02;
constexpr int tiltControllerI = 0.0;
constexpr int tiltControllerD = 0.0;

constexpr int armCurrentLimit = 20;

constexpr float encoderToArmRatio = 12.0 / 54.0;
constexpr float armMinFromVertical = 84.0;
constexpr float shooterToArmAngle = 46.0;

class Arm 
{
private:
    rev::CANSparkMax leftSideLead = rev::CANSparkMax(leftFrontMotorID, rev::CANSparkBase::MotorType::kBrushless);
    rev::CANSparkMax leftSideFollow = rev::CANSparkMax(leftBackMotorID, rev::CANSparkBase::MotorType::kBrushless);
    rev::CANSparkMax rightSideLead = rev::CANSparkMax(rightFrontMotorID, rev::CANSparkBase::MotorType::kBrushless);
    rev::CANSparkMax rightSideFollow = rev::CANSparkMax(rightBackMotorID, rev::CANSparkBase::MotorType::kBrushless);

    frc::DutyCycleEncoder tiltEncoder = frc::DutyCycleEncoder(0);
    frc::PIDController tiltController{tiltControllerP, tiltControllerI, tiltControllerD};

    bool stopTiltMotor = true;
    double tiltSetpoint;
    float maxTiltSetpoint = 61;
    void setAllMotors(double input);


public:
    enum armPosition {
        HIGH, STOW
    };
    void init();
    void runPeriodic();
    void enableMotors();
    void disableMotors();

    Rotation2d getAxleAngle();
    Rotation2d getShooterAngle();
    void setPosition(float desiredAngle);
    void setPosition(armPosition desiredPosition);


};