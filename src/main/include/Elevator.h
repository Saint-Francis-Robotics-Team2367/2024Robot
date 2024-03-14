// #pragma once

// #include <rev/CANSparkMax.h>
// #include <frc/smartdashboard/SmartDashboard.h>

// constexpr int leftElevatorID = 9;
// constexpr int rightElevatorID = 18;

// class Elevator
// {

// public:
//     void init();
//     double lowestSetpoint = 0.0;
//     double highSetpoint = 45;
//     enum elevatorState {
//         STOW, HIGH
//     };
//     elevatorState currentState = STOW;
//     void setState(elevatorState state);
//     void setPID(double kP, double kI, double kD, double kFF, double min, double max, int slot);


//     rev::CANSparkMax motorLeft = rev::CANSparkMax(leftElevatorID, rev::CANSparkLowLevel::MotorType::kBrushless);
//     rev::CANSparkMax motorRight = rev::CANSparkMax(rightElevatorID, rev::CANSparkLowLevel::MotorType::kBrushless);
//     rev::SparkRelativeEncoder leftEnc = motorLeft.GetEncoder();
//     rev::SparkRelativeEncoder rightEnc = motorRight.GetEncoder();
//     rev::SparkPIDController leftController = motorLeft.GetPIDController();
//     rev::SparkPIDController rightController = motorRight.GetPIDController();
// };