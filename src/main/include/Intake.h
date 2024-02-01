#pragma once
#include <rev/CANSparkMax.h>

#define intakeMotorID 19

class Intake
{
private:
    int intakeCurrentLimit = 10;
    rev::CANSparkMax *intakeMotor = new rev::CANSparkMax(intakeMotorID, rev::CANSparkMax::MotorType::kBrushed);
    void calculateIntakeVelocity(double robotVelocityLinear);
    
public:
    void init();
    void intake(double percentSpeed);
    void exhaust(double percentSpeed);
    bool noteDetected();
};