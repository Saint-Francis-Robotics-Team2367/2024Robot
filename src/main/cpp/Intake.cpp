#include "Intake.h"

void Intake::init()
{
    intakeMotor.RestoreFactoryDefaults();
    intakeMotor.SetSmartCurrentLimit(intakeCurrentLimit);
    intakeMotor.SetInverted(true);
    intakeMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
    setPID(velocityP, velocityI, velocityD, velocityFF, -1.0, 1.0);
}

void Intake::disable()
{
    intakeMotor.StopMotor();
    currentState = intakeState::STOP;
}

void Intake::setIntakeSpeed(double speed)
{
    intakeSpeed = speed;
}

void Intake::setIntakeState(intakeState state)
{
    if (state == currentState) {
        return;
    } else {
        currentState = state;
    }
    switch (state)
    {
    case IN:
        // intakeMotor.SetSmartCurrentLimit(intakeCurrentLimit);
        // intakeMotor.Set(1.0);
        intakeController.SetReference(intakeSpeed, rev::CANSparkBase::ControlType::kVelocity);
        break;
    case CLEAR:
        clear();
        break;
    case STOP:
        disable();
        break;
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
        // intakeMotor.SetSmartCurrentLimit(clearingCurrentLimit);
        intakeController.SetReference(-intakeSpeed, rev::CANSparkBase::ControlType::kVelocity);
    }
    // intakeMotor.Set(-1.0);
}

void Intake::setPID(double kP, double kI, double kD, double kFF, double min, double max)
{
    intakeController.SetP(kP);
    intakeController.SetI(kI);
    intakeController.SetD(kD);
    intakeController.SetFF(kFF);

    intakeController.SetOutputRange(min, max);


}