#include "Elevator.h"

void Elevator::init()
{
    motorLeft.RestoreFactoryDefaults();
    motorRight.RestoreFactoryDefaults();

    motorLeft.SetSmartCurrentLimit(60);
    motorRight.SetSmartCurrentLimit(60);

    motorLeft.SetInverted(true);
    motorRight.SetInverted(true);

    motorLeft.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    motorRight.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);

    leftEnc.SetPosition(highSetpoint);
    rightEnc.SetPosition(highSetpoint);

    setPID(0.2, 0.0005, 0.0, 0.0, -1.0, 1.0, 0);

}

void Elevator::setState(elevatorState state)
{
    if (state != currentState) 
    {
        switch (state) 
        {
            case STOW:
                leftController.SetReference(lowestSetpoint, rev::CANSparkBase::ControlType::kPosition);
                rightController.SetReference(lowestSetpoint, rev::CANSparkBase::ControlType::kPosition);
                break;
            case HIGH:
                leftController.SetReference(highSetpoint, rev::CANSparkBase::ControlType::kPosition);
                rightController.SetReference(highSetpoint, rev::CANSparkBase::ControlType::kPosition);
                break;
        }

    }
}

void Elevator::setPID(double kP, double kI, double kD, double kFF, double min, double max, int slot)
{
    leftController.SetP(kP, slot);
    leftController.SetI(kI, slot);
    leftController.SetD(kD, slot);
    leftController.SetFF(kFF, slot);

    leftController.SetOutputRange(min, max, slot);

    rightController.SetP(kP, slot);
    rightController.SetI(kI, slot);
    rightController.SetD(kD, slot);
    rightController.SetFF(kFF, slot);

    rightController.SetOutputRange(min, max, slot);
}