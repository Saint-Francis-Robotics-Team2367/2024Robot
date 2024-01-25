#include "Climber.h"
#define maxSmartCurrent 60

void Climber::moveManual(double leftInput, double rightInput) { 
    desiredLeftHeight = leftInput*maxHeight;
    desiredRightHeight = rightInput*maxHeight;

    leftController.SetReference(desiredLeftHeight, rev::ControlType::kPosition);
    rightController.SetReference(desiredRightHeight, rev::ControlType::kPosition);
}

void Climber::init() { //resets motors and encoders settings
    leftClimberMotor->RestoreFactoryDefaults();
    rightClimberMotor->RestoreFactoryDefaults();

    leftClimberMotor->SetInverted(true);
    rightClimberMotor->SetInverted(true);

    leftClimberMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    rightClimberMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    leftClimberMotor->SetSmartCurrentLimit(maxSmartCurrent);
    rightClimberMotor->SetSmartCurrentLimit(maxSmartCurrent);
}

void Climber::moveToMaxHeight() {
    desiredLeftHeight = maxHeight;
    desiredRightHeight = maxHeight; 

    leftController.SetReference(desiredLeftHeight, rev::ControlType::kPosition);
    rightController.SetReference(desiredRightHeight, rev::ControlType::kPosition);
}

void Climber::moveToMinHeight() {
    desiredLeftHeight = minHeight;
    desiredRightHeight = minHeight; 

    leftController.SetReference(desiredLeftHeight, rev::ControlType::kPosition);
    rightController.SetReference(desiredRightHeight, rev::ControlType::kPosition);
}

bool Climber::isAtSetpoint() {
    if (abs(desiredLeftHeight - leftEnc.GetPosition()) < 0.05) {
        return true;
    }
    return false;
}