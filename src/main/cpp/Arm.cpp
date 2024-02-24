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

    
    stopTiltMotor = true;
    tiltEncoder.Reset();
    tiltEncoder.Reset();
}

void Arm::zeroSensors() {
    tiltEncoder.Reset();
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

void Arm::runPeriodic()
{
    frc::SmartDashboard::PutNumber("setpt", tiltSetpoint);
    frc::SmartDashboard::PutNumber("ShooterRun", getShooterAngle().getDegrees());
    double output = -tiltController.Calculate(getShooterAngle().getDegrees(), tiltSetpoint);
    frc::SmartDashboard::PutNumber("PIDout", output);
    output = std::clamp(output, -0.15, 0.25);
    setAllMotors(output);
}

void Arm::disable()
{
    leftSideLead.StopMotor();
    leftSideFollow.StopMotor();
    rightSideLead.StopMotor();
    rightSideFollow.StopMotor();
}

void Arm::setPosition(float desiredAngle) // setpoint in degrees
{
    if (desiredAngle <= maxTiltSetpoint)
    {
        tiltSetpoint = desiredAngle; // converted to revolutions
    }
}

void Arm::setPosition(armPosition desiredPosition)
{
    double setpoint;
    switch (desiredPosition)
    {
    case HIGH:
        setpoint = highSetpoint;
        break;
    case STOW:
        setpoint = stowSetpoint;
        break;
    }
    tiltSetpoint = setpoint;
}

void Arm::incrementPosition(float increment)
{
    setPosition(tiltSetpoint + increment);
}