#include "Index.h"

void Index::init()
{
    indexMotor.RestoreFactoryDefaults();
    indexMotor.ClearFaults();
    indexMotor.SetSmartCurrentLimit(indexCurrentLimit);
    indexMotor.SetInverted(true);
    // setPID(velocityP, velocityI, velocityD, velocityFF, -1.0, 1.0, VELOCITY);
    // setPID(0.2, positionI, positionD, positionFF, -0.8, 0.8, POSITION);
}

void Index::disable()
{
    indexMotor.StopMotor();
    velocitySetpoint = 0.0;
}

void Index::setVelocity(double velocity)
{
    // inDistanceMode = false;

    // if (velocity != velocitySetpoint)
    // {
    //     velocitySetpoint = velocity;
    //     indexController.SetReference(velocitySetpoint, rev::CANSparkLowLevel::ControlType::kVelocity, VELOCITY);
    // }
    indexMotor.Set(velocity / fabs(velocity));
}

// void Index::setPID(double kP, double kI, double kD, double kFF, double min, double max, int slot)
// {
//     indexController.SetP(kP, slot);
//     indexController.SetI(kI, slot);
//     indexController.SetD(kD, slot);
//     indexController.SetFF(kFF, slot);

//     indexController.SetOutputRange(min, max, slot);
// }

// void Index::setDistance(double distance)
// {
//     inDistanceMode = true;
//     indexEncoder.SetPosition(0.0);
//     distanceSetpoint = distance;
//     indexController.SetReference(distanceSetpoint, rev::CANSparkLowLevel::ControlType::kPosition, POSITION);
// }

// bool Index::isDistanceFinished(float percentageBound)
// {
//     if (inDistanceMode)
//     {
//         double pos = indexEncoder.GetPosition();
//         return (pos < (distanceSetpoint * (1 + percentageBound))) && (pos > (distanceSetpoint * (1 - percentageBound)));
//     }
//     else
//     {
//         return false;
//     }
// }

int Index::getSensorProximity()
{
    return colorSensor.GetProximity();
}

bool Index::isNoteDetected()
{
    bool blueLess = colorSensor.GetColor().blue < 0.15;
    bool redCheck = colorSensor.GetColor().red > 0.40;
    return blueLess && redCheck;
}
