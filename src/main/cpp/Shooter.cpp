#include "Shooter.h"

void Shooter::initShooter()
{
    bottomMotor->RestoreFactoryDefaults();
    topMotor->RestoreFactoryDefaults();

    bottomMotor->SetSmartCurrentLimit(20);
    topMotor->SetSmartCurrentLimit(20);
    tiltMotor->SetSmartCurrentLimit(10);

    bottomMotor->SetInverted(true); // check later
    topMotor->SetInverted(false);
    tiltMotor->SetInverted(false);

    tiltController.SetP(anglekP); // change these
    tiltController.SetI(anglekI);
    tiltController.SetD(anglekD);

    topShooterController.SetP(shooterkP);
    topShooterController.SetI(shooterkI);
    topShooterController.SetD(shooterkD);

    bottomShooterController.SetP(shooterkP);
    bottomShooterController.SetI(shooterkI);
    bottomShooterController.SetD(shooterkD);

    motorThread = std::thread(&run, this);
}

void Shooter::run()
{
    while (true)
    {
        if (stopShooterMotor == true)
        {
            bottomMotor->StopMotor();
            topMotor->StopMotor();
        }
        else
        {
            bottomShooterController.SetReference(shooterVelocitySetpoint, rev::ControlType::kVelocity);
            topShooterController.SetReference(shooterVelocitySetpoint, rev::ControlType::kVelocity);
        }

        if (stopTiltMotor)
        {
            tiltMotor->StopMotor();
        }
        else
        {
            tiltController.SetReference(tiltSetpoint, rev::ControlType::kPosition);
        }
    }
}

void Shooter::stopMotors()
{
    stopTiltMotor = true;
    stopShooterMotor = true;
}

void Shooter::enableMotors()
{
    stopTiltMotor = false;
    stopShooterMotor = false;
    // reset tilt setpoint to avoid jumping
}

void Shooter::setAngleSetpoint(float setpoint)
{
    tiltSetpoint = setpoint;
}

void Shooter::setMotorVelocitySetpoint(float setpoint)
{
    shooterVelocitySetpoint = setpoint;
}

double Shooter::getMotorVelocity()
{
    return topShooterEncoder.GetVelocity();
}

double Shooter::getAnglePosition()
{
    return tiltEncoder.GetPosition();
}