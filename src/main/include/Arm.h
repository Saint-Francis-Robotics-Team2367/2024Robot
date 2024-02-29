#pragma once

#include <rev/CANSparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DutyCycleEncoder.h>
#include "geometry/Rotation2d.h"
#include <frc/controller/PIDController.h>
#include <rev/SparkPIDController.h>
#include <rev/SparkRelativeEncoder.h>
#include <thread>
#include "Constants.h"
#include <thread>

constexpr float tiltControllerP = 0.02;
constexpr float tiltControllerI = 0.0;
constexpr float tiltControllerD = 0.0;

constexpr int armCurrentLimit = 20;

constexpr float encoderToArmRatio = 12.0 / 54.0;
constexpr float armMinFromVertical = 67.2;
constexpr float shooterToArmAngle = 46.0;

class Arm
{
public:
    rev::CANSparkMax leftSideLead = rev::CANSparkMax(motorIDs::leftFrontMotorID, rev::CANSparkBase::MotorType::kBrushless);
    rev::CANSparkMax leftSideFollow = rev::CANSparkMax(motorIDs::leftBackMotorID, rev::CANSparkBase::MotorType::kBrushless);
    rev::CANSparkMax rightSideLead = rev::CANSparkMax(motorIDs::rightFrontMotorID, rev::CANSparkBase::MotorType::kBrushless);
    rev::CANSparkMax rightSideFollow = rev::CANSparkMax(motorIDs::rightBackMotorID, rev::CANSparkBase::MotorType::kBrushless);

    frc::DutyCycleEncoder tiltEncoder = frc::DutyCycleEncoder(0);
    frc::PIDController tiltController{tiltControllerP, tiltControllerI, tiltControllerD};

    double tiltSetpoint;
    double lastTiltSetpoint;
    float maxTiltSetpoint = 70;
    void setAllMotors(double input);

    // Preset Positions
    const float highSetpoint = -45.0;
    float stowSetpoint;

    const float speakerHeight = 2.045081;// meters
    const double rollerCircumference = 0.31918581360576; // meters

public:
    enum armPosition
    {
        HIGH,
        STOW
    };
    void init();
    void runPeriodic();
    void disable();
    void zeroSensors();

    Rotation2d getAxleAngle();
    Rotation2d getShooterAngle();
    void setPosition(float desiredAngle);
    void setPosition(armPosition desiredPosition);
    void incrementPosition(float increment);
    double heightAtAngle(double velocity, double x, double y);
    double findLaunchAngle(double velocity, double x, double y);
};