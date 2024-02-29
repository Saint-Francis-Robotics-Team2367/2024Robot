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
    
    setPID(positionP, positionI, positionD, positionFF, -0.8, 0.8, POSITION);
    setPID(velocityP, velocityI, velocityD, velocityFF, -1.0, 1.0, VELOCITY);
}

void Shooter::disable()
{
    topRollerMotor.StopMotor();
    bottomRollerMotor.StopMotor();
}

void Shooter::setSpeed(float rotationsPerMinute)
{
    
    if (rotationsPerMinute != velocitySetpoint) {
        inDistanceMode = false;
        velocitySetpoint = rotationsPerMinute;
        if (rotationsPerMinute <= maxVelocitySetpoint)
        {
            topRollerController.SetReference(rotationsPerMinute, rev::CANSparkLowLevel::ControlType::kVelocity, VELOCITY);
            bottomRollerController.SetReference(rotationsPerMinute, rev::CANSparkLowLevel::ControlType::kVelocity, VELOCITY);
        }
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
        topRollerMotor.StopMotor();
        bottomRollerMotor.StopMotor();
        velocitySetpoint = 0.0;
        break;
    }
}

void Shooter::setDistance(float distance) 
{
    velocitySetpoint = 0.0;
    inDistanceMode = true;
    topRollerEncoder.SetPosition(0.0);
    bottomRollerEncoder.SetPosition(0.0);
    distanceSetpoint = distance;
    topRollerController.SetReference(distanceSetpoint, rev::CANSparkLowLevel::ControlType::kPosition, POSITION);
    bottomRollerController.SetReference(distanceSetpoint, rev::CANSparkLowLevel::ControlType::kPosition, POSITION);
}

bool Shooter::isDistanceFinished(float percentageBound)
{
    double pos = topRollerEncoder.GetPosition();
    return (pos < (distanceSetpoint * (1 + percentageBound))) && (pos > (distanceSetpoint * (1 - percentageBound)));
}

double Shooter::getSpeed() {
    return topRollerEncoder.GetVelocity();
}

void Shooter::setPID(double kP, double kI, double kD, double kFF, double min, double max, int slot)
{
    topRollerController.SetP(kP, slot);
    topRollerController.SetI(kI, slot);
    topRollerController.SetD(kD, slot);
    topRollerController.SetFF(kFF, slot);

    topRollerController.SetOutputRange(min, max, slot);

    bottomRollerController.SetP(kP, slot);
    bottomRollerController.SetI(kI, slot);
    bottomRollerController.SetD(kD, slot);
    bottomRollerController.SetFF(kFF, slot);

    bottomRollerController.SetOutputRange(min, max, slot);


}
