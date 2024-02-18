#include "Arm.h"

void Arm::init()
{
    leftSideLead.RestoreFactoryDefaults();
    leftSideFollow.RestoreFactoryDefaults();
    rightSideLead.RestoreFactoryDefaults();
    rightSideFollow.RestoreFactoryDefaults();

    leftSideLead.SetInverted(true);
    leftSideFollow.SetInverted(true);

    leftSideLead.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    leftSideLead.SetSmartCurrentLimit(armCurrentLimit);
    leftSideFollow.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    leftSideFollow.SetSmartCurrentLimit(armCurrentLimit);
    rightSideLead.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    rightSideLead.SetSmartCurrentLimit(armCurrentLimit);
    rightSideFollow.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    rightSideFollow.SetSmartCurrentLimit(armCurrentLimit);

    tiltEncoder.Reset();
    stopTiltMotor = true;
    motorThread = std::thread(&Arm::run, this);
}

void Arm::setAllMotors(double input)
{
    leftSideLead.Set(input);
    rightSideLead.Set(input);
    rightSideFollow.Set(input);
    leftSideFollow.Set(input);
}

Rotation2d Arm::getAxleAngle()
{
    return Rotation2d(tiltEncoder.Get().value() * M_PI * 2 * encoderToArmRatio);
}

Rotation2d Arm::getShooterAngle()
{
    return Rotation2d((M_PI_2) - (getAxleAngle().getRadians() + (armMinFromVertical * M_PI / 180)) + (shooterToArmAngle * M_PI / 180));
}

void Arm::run()
{
    while (true)
    {
        if (stopTiltMotor == true)
        {
            leftSideLead.StopMotor();
            leftSideFollow.StopMotor();
            rightSideLead.StopMotor();
            rightSideFollow.StopMotor();
        }
        else
        {
            double output = -tiltController.Calculate(getShooterAngle().getDegrees(), tiltSetpoint);
            output = std::clamp(output, -0.06, 0.1);
            setAllMotors(output);
            frc::SmartDashboard::PutNumber("PIDout", output);
        }
    }
}

void Arm::disableMotors()
{
    stopTiltMotor = true;
}

void Arm::enableMotors()
{
    stopTiltMotor = false;
}

void Arm::setAngleSetpoint(float setpoint) // setpoint in degrees
{
    if (setpoint <= maxTiltSetpoint)
    {
        tiltSetpoint = setpoint; // converted to revolutions
    }
}

void Arm::setAngleSetpoint(armPosition desiredPosition)
{
    float setpoint;
    switch(desiredPosition) 
    {
        case HIGH:
            setpoint = -20.0;
            break;
        case STOW:
            setpoint = 60.0;
            break;
    }
    tiltSetpoint = setpoint;
}