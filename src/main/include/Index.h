#pragma once

#include <rev/SparkPIDController.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/ColorSensorV3.h>
#include "Constants.h"

constexpr int indexCurrentLimit = 10;

class Index
{
public:
    float distanceSetpoint;
    float velocitySetpoint;

    bool inDistanceMode = false;
    rev::CANSparkMax indexMotor = rev::CANSparkMax(motorIDs::indexMotorID, rev::CANSparkLowLevel::MotorType::kBrushless);
    rev::SparkPIDController indexController = indexMotor.GetPIDController();
    rev::SparkRelativeEncoder indexEncoder = indexMotor.GetEncoder();
    rev::ColorSensorV3 colorSensor = rev::ColorSensorV3(frc::I2C::Port::kOnboard);

public:
    void init();
    void disable();
    void setVelocity(double velocity);
    void setDistance(double distance);
    bool isDistanceFinished(float percentageBound);
    int getSensorProximity();
};
