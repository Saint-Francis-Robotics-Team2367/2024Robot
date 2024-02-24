#pragma once
#include <rev/CANSparkMax.h>
#include <rev/SparkPIDController.h>
#include <rev/SparkRelativeEncoder.h>
#include <string>
#include <util/ShuffleUI.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Constants.h"

constexpr unsigned int clearingCurrentLimit = 60;
constexpr unsigned int intakeCurrentLimit = 10;

class Intake
{
private:
    int intakeSpeed = 5700;

    // Clear Thresholds
    const int clearCurrentThreshold = 40;
    const int clearVelocityThreshold = 100;

    rev::CANSparkMax intakeMotor = rev::CANSparkMax(motorIDs::intakeMotorID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkPIDController intakeController = intakeMotor.GetPIDController();
    rev::SparkRelativeEncoder intakeEncoder = intakeMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

    void clear();

public:
    enum intakeState
    {
        IN,
        CLEAR,
        STOP
    };

    void init();
    void disable();
    void setIntakeState(intakeState state);
    void setIntakeSpeed(double speed);


};