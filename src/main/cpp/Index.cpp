#include "Index.h"

void Index::init()
{
    indexMotor.RestoreFactoryDefaults();
    indexMotor.ClearFaults();
    indexMotor.SetSmartCurrentLimit(indexCurrentLimit);
}

void Index::disable()
{
    indexMotor.StopMotor();
}

void Index::setVelocity(double velocity)
{
    inDistanceMode = false;
    velocitySetpoint = velocity;
    indexController.SetReference(velocitySetpoint, rev::CANSparkLowLevel::ControlType::kVelocity);
}

void Index::setDistance(double distance)
{
    inDistanceMode = true;
    indexEncoder.SetPosition(0.0);
    distanceSetpoint = distance;
    indexController.SetReference(distanceSetpoint, rev::CANSparkLowLevel::ControlType::kPosition);
}

bool Index::isDistanceFinished(float percentageBound)
{
    double pos = indexEncoder.GetPosition();
    return (pos < (distanceSetpoint * (1 + percentageBound))) && (pos > (distanceSetpoint * (1 - percentageBound)));
}

int Index::getSensorProximity() {
    return colorSensor.GetProximity();
}
