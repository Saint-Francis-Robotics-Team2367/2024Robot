#pragma once

#include "sensors/Limelight.h"
#include "geometry/Rotation2d.h"
#include "geometry/Translation2d.h"
#include <rev/CANSparkMax.h>
#include <thread>
#include <rev/SparkPIDController.h>
#include <rev/SparkRelativeEncoder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Constants.h"



constexpr float shooterCurrentLimit = 20;

class Shooter
{
public:
    rev::CANSparkMax topRollerMotor = rev::CANSparkMax(motorIDs::topRollerID, rev::CANSparkBase::MotorType::kBrushless);
    rev::CANSparkMax bottomRollerMotor = rev::CANSparkMax(motorIDs::bottomRollerID, rev::CANSparkBase::MotorType::kBrushless);

    rev::SparkPIDController topRollerController = topRollerMotor.GetPIDController();
    rev::SparkPIDController bottomRollerController = bottomRollerMotor.GetPIDController();

    rev::SparkRelativeEncoder topRollerEncoder = topRollerMotor.GetEncoder();
    rev::SparkRelativeEncoder bottomRollerEncoder = bottomRollerMotor.GetEncoder();

    const float maxVelocitySetpoint = 6000.0;
    const float lowVelocitySetpoint = 500.0;
    float distanceSetpoint;
    double velocitySetpoint = 0.0;
    bool inDistanceMode = false;

public:
    enum shooterSpeeds
    {
        HIGH,
        LOW,
        STOP
    };

    void setPID(double kP, double kI, double kD, double kFF, double min, double max);
    void init();
    void disable();
    void setSpeed(shooterSpeeds speed);
    void setSpeed(float rotationsPerMinute);
    void setDistance(float distance);
    bool isDistanceFinished(float percentageBound);
    double getSpeed();
};
