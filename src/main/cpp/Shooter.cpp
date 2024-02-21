#include "Shooter.h"

void Shooter::init()
{
    topRollerMotor.RestoreFactoryDefaults();
    bottomRollerMotor.RestoreFactoryDefaults();

    topRollerMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
    topRollerMotor.SetSmartCurrentLimit(shooterCurrentLimit);
    bottomRollerMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
    bottomRollerMotor.SetSmartCurrentLimit(shooterCurrentLimit);

    topRollerController.SetP(shooterP);
    topRollerController.SetI(shooterI);
    topRollerController.SetD(shooterD);
    topRollerController.SetFF(shooterFF);

    bottomRollerController.SetP(shooterP);
    bottomRollerController.SetI(shooterI);
    bottomRollerController.SetD(shooterD);
    bottomRollerController.SetFF(shooterFF);
}

void Shooter::disable()
{
    topRollerMotor.StopMotor();
    bottomRollerMotor.StopMotor();
}

void Shooter::setSpeed(float rotationsPerMinute)
{
    if (rotationsPerMinute <= maxVelocitySetpoint)
    {
        topRollerController.SetReference(rotationsPerMinute, rev::CANSparkLowLevel::ControlType::kVelocity);
        bottomRollerController.SetReference(rotationsPerMinute, rev::CANSparkLowLevel::ControlType::kVelocity);
    }
}

void Shooter::setSpeed(shooterSpeeds speed)
{
    switch (speed)
    {
    case HIGH:
        setSpeed(maxVelocitySetpoint);
        break;
    case LOW:
        setSpeed(lowVelocitySetpoint);
        break;
    case STOP:
        setSpeed(0.0);
        break;
    }
}

void Shooter::setDistance(float distance) 
{
    topRollerEncoder.SetPosition(0.0);
    bottomRollerEncoder.SetPosition(0.0);
    distanceSetpoint = distance;
    topRollerController.SetReference(distanceSetpoint, rev::CANSparkLowLevel::ControlType::kPosition);
    bottomRollerController.SetReference(distanceSetpoint, rev::CANSparkLowLevel::ControlType::kPosition);
}

bool Shooter::isDistanceFinished(float percentageBound)
{
    double pos = topRollerEncoder.GetPosition();
    return (pos < (distanceSetpoint * (1 + percentageBound))) && (pos > (distanceSetpoint * (1 - percentageBound)));
}
