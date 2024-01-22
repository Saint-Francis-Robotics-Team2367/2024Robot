#include "Shooter.h"

void Shooter::initShooter() {
    bottomMotor->RestoreFactoryDefaults();
    topMotor->RestoreFactoryDefaults();
    bottomMotor->SetInverted(true); //check later
    topMotor->SetInverted(false);

    angleControl.SetP(anglekP); //change these
    angleControl.SetI(anglekI);
    angleControl.SetD(anglekD);
    shooterControl.SetP(shooterkP);
    shooterControl.SetI(shooterkI);
    shooterControl.SetD(shooterkD);
    frc::SmartDashboard::PutNumber("P Gain", shooterkP);
    frc::SmartDashboard::PutNumber("I Gain", shooterkI);
    frc::SmartDashboard::PutNumber("D Gain", shooterkD);
    frc::SmartDashboard::PutNumber("P Gain - angle", anglekP);
    frc::SmartDashboard::PutNumber("I Gain - angle", anglekI);
    frc::SmartDashboard::PutNumber("D Gain - angle", anglekD);

    shooterThread = std::thread(&run, this);


}

void Shooter::run() {

    if (stopShooterMotor==true) {
        bottomMotor->StopMotor();
        topMotor->StopMotor();
    }
    else if (stopAngleMotor==true) {
        angleMotor->StopMotor();
    }
    else {
        enableMotors();
        setAngleSetpoint(0);
        setMotorVelocitySetpoint(0);

    }
}

void Shooter::stopMotors() {
    stopMotor = true;
}

void Shooter::enableMotors() {
    stopMotor = false;
}

void Shooter::setAngleSetpoint(float setpoint) {//setpoint in degrees
    setpoint = setpoint/360; //converting to revolutions

    double p = frc::SmartDashboard::GetNumber("P Gain - angle", 0);
    double i = frc::SmartDashboard::GetNumber("I Gain - angle", 0);
    double d = frc::SmartDashboard::GetNumber("D Gain - angle", 0);

    if(p != anglekP) {
        angleControl.SetP(p);
        anglekP = p;
    }
    if(i != anglekI) {
        angleControl.SetI(i);
        anglekI = i;
    }
    if(d != anglekD) {
        angleControl.SetD(d);
        anglekD = d;
    }

    angleControl.SetReference(setpoint, rev::CANSparkMax::ControlType::kPosition);

}

void Shooter::setMotorVelocitySetpoint(float setpoint) {
    double p = frc::SmartDashboard::GetNumber("P Gain", 0);
    double i = frc::SmartDashboard::GetNumber("I Gain", 0);
    double d = frc::SmartDashboard::GetNumber("D Gain", 0);
   
    if(p != shooterkP) {
        shooterControl.SetP(p);
        shooterkP = p;
    }
    if(i != shooterkI) {
        shooterControl.SetI(i);
        shooterkI = i;
    }
    if(d != shooterkD) {
        shooterControl.SetD(d);
        shooterkD = d;
    }

    shooterControl.SetReference(setpoint, rev::CANSparkMax::ControlType::kVelocity);
    frc::SmartDashboard::PutNumber("SetPoint", setpoint);
    frc::SmartDashboard::PutNumber("ProcessVariable", shooterEncoder.GetVelocity());
}

double Shooter::getShooterAngle() {
    double anglePosition = (angleEncoder.GetPosition()*(2*PI)); //converted to radians
    return anglePosition;
}

bool Shooter::isFinished(float percentageBound){
    double velocity = shooterEncoder.GetVelocity();
    bool reachedVelocity = (velocity < (motorVelocitySetpoint * (1 + percentageBound))) && (velocity > (motorVelocitySetpoint * (1 - percentageBound)));

    double angle = getShooterAngle();
    bool reachedAngle = (angle < (angleSetpoint * (1 + percentageBound))) && (angle > (angleSetpoint * (1 - percentageBound)));

    return (reachedAngle && reachedVelocity);
}



/*
// Set the setpoint (target angle in encoder units)
    double targetAngle = 90.0; // Replace with your desired target angle
    armPIDController.SetReference(targetAngle, rev::ControlType::kPosition);

    // Loop to periodically update the control
    while (true) {
        // You can add other logic or wait time here
        // ...

        // Read current encoder position
        double currentAngle = armEncoder.GetPosition();

        // Print current and target angles for debugging
        std::cout << "Current Angle: " << currentAngle << ", Target Angle: " << targetAngle << std::endl;

        // You may want to add a condition to break out of the loop when the arm reaches the target
        // if (fabs(currentAngle - targetAngle) < tolerance) {
        //     break;
        // }
    }

    // Disable the PID controller
    armPIDController.SetReference(0, rev::ControlType::kVoltage);

    return 0;
}*/





/*
float SwerveModule::getSteerAngleSetpoint()
{
    return steerAngleSetpoint;
}

/* Takes in input from 0 - 2pi
 * 0 is the right, goes counterclockwise
 * Not tested
 */
/*void SwerveModule::setSteerAngleSetpoint(float setpt)
{
    steerAngleSetpoint = setpt;
}

/**
 * Untested, I've never used it
 */
/*void SwerveModule::setDrivePositionSetpoint(float setpt)
{
    drivePositionSetpoint = setpt;
    driveMode = POSITION;
}

/**
 * Set the drive motor velocity setpoint to input RPM
 * Max RPM is 5700
 */
/*void SwerveModule::setDriveVelocitySetpoint(float setpt)
{
    driveVelocitySetpoint = setpt;
    driveMode = VELOCITY;
}
*/