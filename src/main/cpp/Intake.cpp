#include "Intake.h"

void Intake::init()
{
    intakeMotor->RestoreFactoryDefaults();
    intakeMotor->SetSmartCurrentLimit(intakeCurrentLimit);
    intakeMotor->SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    currentState = intakeState::STOP;
}

std::string Intake::getEnumString(intakeState state)
{
    switch (state)
    {
    case IN:
        return "IN";
    case OUT:
        return "OUT";
    case STOP:
        return "STOP";
    }
}

void Intake::setState(intakeState state, bool printState, double power)
{
    switch (state)
    {
    case IN:
        intake(power);
        break;
    case OUT:
        exhaust(power);
        break;
    case STOP:
        stop();
        break;
    }
    if (printState)
    {
        ShuffleUI::MakeWidget("IntakeMode", "Intake", state);
    }
}

void Intake::intake(double percentSpeed)
{
    intakeMotor->Set(percentSpeed);
}

void Intake::exhaust(double percentSpeed)
{
    intakeMotor->Set(-percentSpeed);
}

void Intake::stop()
{
    intakeMotor->StopMotor();
}

Intake::intakeState Intake::buttonsToState(bool inButton, bool outButton)
{
    if (inButton)
    {
        return intakeState::IN;
    }
    else if (outButton)
    {
        return intakeState::OUT;
    }
    else
    {
        return intakeState::STOP;
    }
}