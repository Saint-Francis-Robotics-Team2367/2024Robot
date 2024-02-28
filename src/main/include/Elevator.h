#pragma once

#include <rev/CANSparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>


class Elevator

{

public:
    void elevatorUpDown(bool toggle);
    void initMotors();
    Elevator(int leftID, int rightID);
    double lowestEncoderThreshold;
    double highestEncoderThreshold;
    //NEED TO DEFINE THESE!!!! ^^^^

    int leftEMotorID;
    int rightEMotorID;

    rev::CANSparkMax *leftElevatorMotor;
    rev::SparkRelativeEncoder leftEnc;
    rev::CANSparkMax *rightElevatorMotor;
    rev::SparkRelativeEncoder rightEnc;

    //circle button
    bool buttonUp;
};