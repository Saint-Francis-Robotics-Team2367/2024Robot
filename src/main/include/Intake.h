#pragma once
#include <rev/CANSparkMax.h>
#include <rev/SparkPIDController.h>
#include <rev/SparkRelativeEncoder.h>
#include <string>
#include <util/ShuffleUI.h>

#define intakeMotorID 19

class Intake
{
private:
    int intakeCurrentLimit = 10;
    double intakeSpeed = 5700;

    void setClearCurrentLimit(double current);
    void setStdCurrentLimit(double current);
    //void calculateIntakeVelocity(double robotVelocityLinear);

    rev::CANSparkMax intakeMotor = rev::CANSparkMax(intakeMotorID, rev::CANSparkMax::MotorType::kBrushed);
    rev::SparkPIDController intakeController = intakeMotor.GetPIDController();
    rev::SparkRelativeEncoder intakeEncoder = intakeMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

public:
    enum Dir
    {
        IN,
        OUT,
        STOP
    };

    void enable();
    void disable();
    void setSpeed(double speed);
    void intake();
    void clear(Dir direction);

    //static intakeState buttonsToState(bool inButton, bool outButton);
    //static std::string getEnumString(intakeState state);
    //intakeState currentState;
    //void setState(intakeState state, bool printState, double power = 1.0);
};