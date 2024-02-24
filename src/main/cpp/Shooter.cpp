#include "Shooter.h"

void Shooter::init()
{
    topRollerMotor.RestoreFactoryDefaults();
    bottomRollerMotor.RestoreFactoryDefaults();

    topRollerMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
    topRollerMotor.SetSmartCurrentLimit(shooterCurrentLimit);
    bottomRollerMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
    bottomRollerMotor.SetSmartCurrentLimit(shooterCurrentLimit);

    topRollerMotor.SetInverted(true);
    bottomRollerMotor.SetInverted(true);
}

void Shooter::disable()
{
    topRollerMotor.StopMotor();
    bottomRollerMotor.StopMotor();
}

void Shooter::setSpeed(float rotationsPerMinute)
{
    setPID(velocityP, velocityI, velocityD, velocityFF, -1.0, 1.0);

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
    setPID(positionP, positionI, positionD, positionFF, -0.8, 0.8);

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

double Shooter::getSpeed() {
    return topRollerEncoder.GetVelocity();
}

void Shooter::setPID(double kP, double kI, double kD, double kFF, double min, double max)
{
    topRollerController.SetP(kP);
    topRollerController.SetI(kI);
    topRollerController.SetD(kD);
    topRollerController.SetFF(kFF);

    topRollerController.SetOutputRange(min, max);

    bottomRollerController.SetP(kP);
    bottomRollerController.SetI(kI);
    bottomRollerController.SetD(kD);
    bottomRollerController.SetFF(kFF);

    bottomRollerController.SetOutputRange(min, max);


}
