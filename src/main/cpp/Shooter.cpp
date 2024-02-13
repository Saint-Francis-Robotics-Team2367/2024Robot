#include "Shooter.h"

void Shooter::init()
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

    topShooterController.SetP(revkP);
    topShooterController.SetI(revkI);
    topShooterController.SetD(revkD);
    topShooterController.SetFF(revkFF);

    bottomShooterController.SetP(revkP);
    bottomShooterController.SetI(revkI);
    bottomShooterController.SetD(revkD);
    bottomShooterController.SetFF(revkFF);

    motorThread = std::thread(&Shooter::run, this);
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
            bottomShooterController.SetReference(shooterVelocitySetpoint, rev::CANSparkBase::ControlType::kVelocity);
            topShooterController.SetReference(shooterVelocitySetpoint, rev::CANSparkBase::ControlType::kVelocity);
        }

        if (stopTiltMotor)
        {
            tiltMotor->StopMotor();
        }
        else
        {
            tiltController.SetReference(tiltSetpoint, rev::CANSparkBase::ControlType::kPosition);
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
    tiltSetpoint = 0;
}

void Shooter::setAngleSetpoint(float setpoint) // setpoint in degrees
{
    if (setpoint <= maxTiltAngle)
    {
        tiltSetpoint = setpoint / 360; // converted to revolutions
    }
}

void Shooter::setMotorVelocitySetpoint(float setpoint)
{
    if (setpoint < maxVelocity)
    {
        shooterVelocitySetpoint = setpoint; // velocity in RPM
    }
}

double Shooter::getMotorVelocity()
{
    return topShooterEncoder.GetVelocity();
}

double Shooter::getAnglePosition()
{
    double anglePosition = (tiltEncoder.GetPosition() * (2 * PI)); // converted to radians
    return anglePosition;
}

double Shooter::heightAtAngle(double shooterVelocity, double x, double theta, double robotVelocity) {
    double thetaRadians = theta * M_PI / 180.0;
    double timeOfFlight = x / ((shooterVelocity * cos(thetaRadians))+robotVelocity);
    double height = shooterVelocity * timeOfFlight * sin(thetaRadians) - 0.5 * 9.81 * pow(timeOfFlight, 2);
    return height;
}

double Shooter::findLaunchAngle(double velocity, double x, double y, Limelight limelight) {
    
    auto func = [&](double theta) {
        return std::abs(heightAtAngle(velocity, x, theta, 0) - y); // REPLACE WITH MDRIVE.GETVELOCITY
    };

    double min_angle = 0.0;
    double max_angle = 90.0;
    double step = 0.01;
    double min_error = std::numeric_limits<double>::max();
    double best_angle = 0.0;

    for (double angle = min_angle; angle <= max_angle; angle += step) {
        double error = func(angle);
        if (error < min_error) {
            min_error = error;
            best_angle = angle;
        }
    }

    return best_angle;
}

void Shooter::runAuto(Limelight limelight) {
    double angle = findLaunchAngle(getMotorVelocity(), limelight.getDistanceToWall(), 0, limelight);

    setAngleSetpoint(angle);
    setMotorVelocitySetpoint(5700); 

    // wait until angles and vel reach desired 
    while(!getMotorVelocity() == 5700 || !getAnglePosition() == angle){

    }

    run();
}