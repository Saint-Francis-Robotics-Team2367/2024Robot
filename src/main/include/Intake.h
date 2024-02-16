#pragma once
#include <rev/CANSparkMax.h>
#include <string>
#include <util/ShuffleUI.h>

#define intakeMotorID 19

class Intake
{
private:
    int intakeCurrentLimit = 10;
    rev::CANSparkMax *intakeMotor = new rev::CANSparkMax(intakeMotorID, rev::CANSparkMax::MotorType::kBrushed);
    void calculateIntakeVelocity(double robotVelocityLinear);

public:
    enum intakeState
    {
        IN,
        OUT,
        STOP
    };
    static intakeState buttonsToState(bool inButton, bool outButton);
    static std::string getEnumString(intakeState state);

    intakeState currentState;
    void init();
    void setState(intakeState state, bool printState, double power = 1.0);
    void intake(double percentSpeed);
    void exhaust(double percentSpeed);
    void stop();
    // bool noteDetected();
};