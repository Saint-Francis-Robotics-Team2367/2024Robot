#include "Intake.h"

void Intake::init()
{
    intakeMotor.RestoreFactoryDefaults();
    intakeMotor.SetSmartCurrentLimit(intakeCurrentLimit);
    intakeMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
}

void Intake::disable()
{
    intakeMotor.StopMotor();
}

void Intake::setIntakeSpeed(double speed)
{
    intakeSpeed = speed;
}

void Intake::setIntakeState(intakeState state)
{
    switch (state)
    {
    case IN:
        intakeMotor.SetSmartCurrentLimit(intakeCurrentLimit);
        intakeController.SetReference(intakeSpeed, rev::CANSparkBase::ControlType::kVelocity);
        break;
    case CLEAR:
        clear();
        break;
    case STOP:
        disable();
    }
}

void Intake::clear()
{
    double motorVelocity = intakeEncoder.GetVelocity();
    double motorCurrentDraw = intakeMotor.GetOutputCurrent();
    if (motorCurrentDraw > clearCurrentThreshold && motorVelocity < clearVelocityThreshold)
    {
        // Report clear failure
        frc::SmartDashboard::PutBoolean("IntakeClearFlag", true);
    }
    else
    {
        frc::SmartDashboard::PutBoolean("IntakeClearFlag", false);
        intakeMotor.SetSmartCurrentLimit(clearingCurrentLimit);
        intakeController.SetReference(-intakeSpeed, rev::CANSparkBase::ControlType::kVelocity);
    }
}