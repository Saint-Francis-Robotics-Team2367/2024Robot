#include "Index.h"

void Index::init()
{
    indexMotor.RestoreFactoryDefaults();
    indexMotor.ClearFaults();
    indexMotor.SetSmartCurrentLimit(indexCurrentLimit);
    indexMotor.SetInverted(true);
    setPID(velocityP, velocityI, velocityD, velocityFF, -1.0, 1.0);
}

void Index::disable()
{
    indexMotor.StopMotor();
}

void Index::setVelocity(double velocity)
{
    inDistanceMode = false;
    if (velocity != velocitySetpoint) 
    {
        velocitySetpoint = velocity;
        indexController.SetReference(velocitySetpoint, rev::CANSparkLowLevel::ControlType::kVelocity);
    }
    // indexMotor.Set(velocity);
    
}

void Index::setPID(double kP, double kI, double kD, double kFF, double min, double max)
{
    indexController.SetP(kP);
    indexController.SetI(kI);
    indexController.SetD(kD);
    indexController.SetFF(kFF);

    indexController.SetOutputRange(min, max);
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

bool Index::isNoteDetected() {
    bool blueLess = colorSensor.GetColor().blue < 0.15;
    bool redCheck = colorSensor.GetColor().red > 0.40;
    return blueLess && redCheck;
}
