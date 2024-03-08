#include "Arm.h"
#include <cmath>

/**
 * Setup motors
 * Sets current limits, brake mode, resets encoder
*/
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
    tiltSetpoint = getShooterAngle().getDegrees();
    stowSetpoint = tiltSetpoint;
}


/**
 * reset tilt encoder
*/
void Arm::zeroSensors() {
    tiltEncoder.Reset();
}

/**
 * Set a value 1.0 from -1.0 to all motors
*/
void Arm::setAllMotors(double input)
{
    leftSideLead.Set(input);
    rightSideLead.Set(input);
    rightSideFollow.Set(input);
    leftSideFollow.Set(input);
}

/**
 * Gets the arm axle angle from a vertical
*/
Rotation2d Arm::getAxleAngle()
{
    return Rotation2d(tiltEncoder.Get().value() * PI * 2 * encoderToArmRatio);
}


/**
 * Get shooter(roller wheels) angle from horizontal
*/
Rotation2d Arm::getShooterAngle()
{
    return Rotation2d((PI_2) - (getAxleAngle().getRadians() + (armMinFromVertical * PI / 180)) + (shooterToArmAngle * PI / 180));
}

/**
 * Run the PID algorithm to tiltSetpoint
 * Function should be placed in a while loop
*/
void Arm::runPeriodic()
{
    frc::SmartDashboard::PutNumber("setpt", tiltSetpoint);
    frc::SmartDashboard::PutNumber("ShooterRun", getShooterAngle().getDegrees());
    double output = -tiltController.Calculate(getShooterAngle().getDegrees(), tiltSetpoint);
    frc::SmartDashboard::PutNumber("PIDout", output);
    output = std::clamp(output, -0.35, 0.45);
    setAllMotors(output);
    
}

/**
 * Stops all motors
*/
void Arm::disable()
{
    leftSideLead.StopMotor();
    leftSideFollow.StopMotor();
    rightSideLead.StopMotor();
    rightSideFollow.StopMotor();
}

/**
 * set the position setpoint to the desired angle in degrees
*/
void Arm::setPosition(float desiredAngle) // setpoint in degrees
{
    if (desiredAngle <= maxTiltSetpoint)
    {
        tiltSetpoint = desiredAngle; // converted to revolutions
    }
}

/**
 * Set the position setpoint to a enum preset
 * HIGH for amp, STOW for lowered
*/
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

/**
 * Increase the tiltSetpoint in degrees by the increment
*/
void Arm::incrementPosition(float increment)
{
    setPosition(tiltSetpoint + increment);
}

double Arm::heightAtAngle(double velocity, double x, double theta)
{
    double theta_rad = theta * PI / 180.0;
    double time_of_flight = x / (velocity * cos(theta_rad));
    double height = velocity * time_of_flight * sin(theta_rad) - 0.5 * 9.81 * pow(time_of_flight, 2);
    return height;
}

double Arm::findLaunchAngle(double velocity, double x, double y)
{

    auto func = [&](double theta)
    {
        return std::abs(heightAtAngle(velocity, x, theta) - y);
    };

    double min_angle = atan(y/x)*(180/PI);
    double max_angle = 90.0;
    double step = 0.01;
    double min_error = std::numeric_limits<double>::max();
    double best_angle = 0.0;

    for (double angle = min_angle; angle <= max_angle; angle += step)
    {
        double error = func(angle);
        if (error < min_error)
        {
            min_error = error;
            best_angle = angle;
        }
    }
    return best_angle;
}

//all measurements given in inches
double Arm::findBetterLaunchAngle(double xTag, double yTag, double zTag){
    //low angle
    int shooterHeight = 18;
    double velocity = 455.59;
    double lowHeight = 78;
    double distToTag2d = sqrt(pow(xTag, 2)+ pow(yTag, 2));
    double distanceLow = sqrt(pow(distToTag2d, 2) + pow(lowHeight-shooterHeight, 2));
    double sinAngleLow = ((lowHeight-shooterHeight)/distanceLow) + ((4.9*distanceLow)/pow(velocity, 2));
    double angleLow = asin(sinAngleLow);
    
    //high angle
    double heightOfTag = 51.875;
    double distToTag3d = sqrt(pow(distToTag2d, 2)+ pow(heightOfTag-shooterHeight, 2));
    double robotTagAngle = asin((heightOfTag-shooterHeight)/distToTag3d);
    int tagToTopX = 16;
    double tagToTopY = 26.55;
    double tagToTopDist = sqrt(pow(tagToTopX, 2)+ pow(tagToTopY, 2));
    double angleTagToTop = asin(tagToTopX/tagToTopDist);
    double angleTagToRobot = 90- robotTagAngle; 
    double angleTopTagRobot = 180-angleTagToRobot-angleTagToTop;

    double distanceRobotToTopSquared = pow(tagToTopDist, 2)+ pow(distToTag3d, 2) - 2*tagToTopDist*distToTag3d*cos(angleTopTagRobot);
    double distanceRobotToTop = sqrt(distanceRobotToTopSquared);
    double heightTop = tagToTopY + heightOfTag;
    double sinAngleHigh = (heightTop-shooterHeight)/distanceRobotToTop + 4.9*distanceRobotToTop/pow(velocity,2);
    double angleHigh = asin(sinAngleHigh);

    return ((angleHigh+angleLow)/2);
    





}
