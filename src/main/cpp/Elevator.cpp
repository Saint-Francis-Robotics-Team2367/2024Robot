#include <Elevator.h>

Elevator::Elevator(int leftID, int rightID) : leftElevatorMotor(new rev::CANSparkMax(leftEMotorID, rev::CANSparkMax::MotorType::kBrushless)),
                                                                    rightElevatorMotor(new rev::CANSparkMax(rightEMotorID, rev::CANSparkMax::MotorType::kBrushless)),
                                                                    leftEnc(leftElevatorMotor->GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42)),
                                                                    rightEnc(rightElevatorMotor->GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42))                                                  
{
    leftEMotorID = leftID;
    rightEMotorID = rightID;
}


void Elevator::initMotors() {
    leftElevatorMotor->RestoreFactoryDefaults();
    rightElevatorMotor->RestoreFactoryDefaults();

    leftElevatorMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    rightElevatorMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    leftElevatorMotor->SetSmartCurrentLimit(20);
    rightElevatorMotor->SetSmartCurrentLimit(20);
    frc::SmartDashboard::PutNumber("leftEnc", leftEnc.GetPosition());
    frc::SmartDashboard::PutNumber("rightEnc", rightEnc.GetPosition());
}

void Elevator::elevatorUpDown(bool toggle){
    if(toggle){
        if (leftEnc.GetPosition() <= lowestEncoderThreshold && rightEnc.GetPosition() <= lowestEncoderThreshold) {
            leftElevatorMotor->Set(1);
            rightElevatorMotor->Set(1);
            if (leftEnc.GetPosition() >= highestEncoderThreshold && rightEnc.GetPosition() >= highestEncoderThreshold) {
                leftElevatorMotor->Set(0);
                rightElevatorMotor->Set(0);
        }
        }
        
    }
    if (toggle){
        if(leftEnc.GetPosition() >= highestEncoderThreshold && rightEnc.GetPosition() >= highestEncoderThreshold){
            leftElevatorMotor->Set(-1);
            rightElevatorMotor->Set(-1);
            if(leftEnc.GetPosition() <= lowestEncoderThreshold && rightEnc.GetPosition() <= lowestEncoderThreshold) {
                leftElevatorMotor->Set(0);
                rightElevatorMotor->Set(0);
            } 
        }

    }
}