#include "Intake.h"

void Intake::enable()
{
    intakeMotor->RestoreFactoryDefaults();
    intakeMotor->SetSmartCurrentLimit(intakeCurrentLimit);
    intakeMotor->SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    //currentState = intakeState::STOP;
}

void Intake::disable() {
    intakeMotor->StopMotor();
}

void Intake::setSpeed(double speed) {
    intakeSpeed = speed;
}

void Intake::intake() {
    intakeController.SetReference(intakeSpeed, rev::CANSparkBase::ControlType::kVelocity);
}

void Intake::clear(Dir direction) {
    if (direction==OUT) {
        intakeMotor->Set(-1);
    }
}

void Intake::setClearCurrentLimit(double current) {
    intakeMotor->SetSmartCurrentLimit(current);
}

void Intake::setStdCurrentLimit(double current) {
    intakeMotor->SetSmartCurrentLimit(current);
}

/*
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
}*/