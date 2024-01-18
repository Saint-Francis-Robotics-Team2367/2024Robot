#include "SwerveModule.h"

SwerveModule::SwerveModule(int steerMotorID, int driveMotorID, int cancoderID) : steerMotor(new rev::CANSparkMax(steerMotorID, rev::CANSparkMax::MotorType::kBrushless)),
                                                                                 driveMotor(new rev::CANSparkMax(driveMotorID, rev::CANSparkMax::MotorType::kBrushless)),
                                                                                 steerEnc(CAN_Coder(cancoderID)),
                                                                                 driveEnc(driveMotor->GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42)),
                                                                                 m_pidController(driveMotor->GetPIDController())
{
    steerID = steerMotorID;
    driveID = driveMotorID;
}

void SwerveModule::initMotors()
{
    // Resetting Motor settings, Encoders, putting it in brake mode
    steerMotor->RestoreFactoryDefaults();
    driveMotor->RestoreFactoryDefaults();

    steerMotor->SetInverted(true);
    driveMotor->SetInverted(true);

    driveEnc.SetPosition(0);

    // Makes motor stiff(coast mode lets it run freely)
    steerMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    driveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    // Keep the motor limit at under 20A
    steerMotor->SetSmartCurrentLimit(maxSteerCurrent);
    driveMotor->SetSmartCurrentLimit(maxDriveCurrent);

    // Conversion factor from Rotations of motor, which is nothing for now
    driveEnc.SetPositionConversionFactor(1.0);

    // Setpoints to initial encoder positions/speeds
    steerAngleSetpoint = steerEnc.getPosition().getRadians();
    driveVelocitySetpoint = 0.0;

    // Set PID values for REV Drive PID
    m_pidController.SetP(kP);
    m_pidController.SetI(kI);
    m_pidController.SetFF(kFF);
    m_pidController.SetOutputRange(kMinOutput, kMaxOutput);
    steerCTR.EnableContinuousInput(0, 2 * M_PI);

    // Clears Motor Controller's Log of errors
    steerMotor->ClearFaults();
    driveMotor->ClearFaults();
}

float SwerveModule::getSteerAngleSetpoint()
{
    return steerAngleSetpoint;
}

/* Takes in input from 0 - 2pi
 * 0 is the right, goes counterclockwise
 * Not tested
 */
void SwerveModule::setSteerAngleSetpoint(float setpt)
{
    steerAngleSetpoint = setpt;
}

/**
 * Untested, I've never used it
 */
void SwerveModule::setDrivePositionSetpoint(float setpt)
{
    drivePositionSetpoint = setpt;
    driveMode = POSITION;
}

/**
 * Set the drive motor velocity setpoint to input RPM
 * Max RPM is 5700
 */
void SwerveModule::setDriveVelocitySetpoint(float setpt)
{
    driveVelocitySetpoint = setpt;
    driveMode = VELOCITY;
}

/**
 * speedMPS attribute should be in RPM
 * Sets Drive Velocity & Steer Angle
 */
void SwerveModule::setModuleState(SwerveModuleState setpt, bool takeShortestPath)
{
    if (takeShortestPath) 
    {
        SwerveModuleState outputs = moduleSetpointGenerator(getModuleState(), setpt);
        setDriveVelocitySetpoint(outputs.getSpeedFPS());
        setSteerAngleSetpoint(outputs.getRot2d().getRadians());
    }
    else {
        driveVelocitySetpoint = setpt.getSpeedFPS();
        steerAngleSetpoint = setpt.getRot2d().getRadians();
    }
    
}

SwerveModuleState SwerveModule::moduleSetpointGenerator(SwerveModuleState currState, SwerveModuleState desiredSetpoint)
{
    // double elapsedTime = aTimer.Get().value();

    double currAngle = currState.getRot2d().getRadians();
    double currVel = currState.getSpeedFPS();
    double desAngle = desiredSetpoint.getRot2d().getRadians();
    double desVel = desiredSetpoint.getSpeedFPS();

    // ControlUtil::limitAcceleration(currVel, desVel, maxDriveAccelerationRPM, elapsedTime);

    double dist = fabs(currAngle - desAngle);
    bool flip = (dist > M_PI_2) && (((M_PI * 2) - dist) > M_PI_2);

    double setpointAngle;
    double setpointVel;

    if (flip)
    {
        setpointAngle = Rotation2d::radiansBound(desAngle + M_PI);

        double angleDist = std::min(fabs(setpointAngle - currAngle), (M_PI * 2) - fabs(setpointAngle - currAngle));

        setpointVel = -(desVel * (-angleDist / M_PI_2) + desVel);
    }
    else
    {
        setpointAngle = desAngle;

        double angleDist = std::min(fabs(setpointAngle - currAngle), (M_PI * 2) - fabs(setpointAngle - currAngle));

        setpointVel = desVel * (-angleDist / M_PI_2) + desVel;
    }
    return SwerveModuleState(setpointVel, Rotation2d(setpointAngle));
}

SwerveModuleState SwerveModule::getModuleState()
{
    double vel = getDriveEncoderVel();
    double angle = steerEnc.getAbsolutePosition().getRadians();

    return SwerveModuleState(vel, angle);
}

/**
 * Unfinished, Untested, pending review
 */
bool SwerveModule::isFinished(float percentageBound)
{
    if (driveMode == POSITION)
    {
        double pos = driveEnc.GetPosition();
        return (pos < (drivePositionSetpoint * (1 + percentageBound))) && (pos > (drivePositionSetpoint * (1 - percentageBound)));
    }
    else
    {
        double pos = driveEnc.GetVelocity();
        return (pos < (driveVelocitySetpoint * (1 + percentageBound))) && (pos > (driveVelocitySetpoint * (1 - percentageBound)));
    }
}

/**
 * This function is meant to run in a while loop
 * when moduleInhibit is true, motors are stopped
 * when moduleInhibit is false, motor PIDs are running
 * steerPID uses frc::PIDController
 * drivePID uses rev::PIDController
 *
 */
void SwerveModule::run()
{
    // TODO: timing loops

    if (moduleInhibit) // Thread is in standby mode
    {

        steerMotor->StopMotor();
        driveMotor->StopMotor();
    }

    else
    {
        // Steer PID
        currentSteerOutput = steerCTR.Calculate(steerEnc.getAbsolutePosition().getRadians(), steerAngleSetpoint);
        steerMotor->Set(currentSteerOutput);

        // Drive Motor uses the internal REV PID, since optimizations here are rarely needed
        if (driveMode == POSITION)
        {
            m_pidController.SetReference(drivePositionSetpoint, rev::CANSparkMax::ControlType::kPosition);
        }
        else
        {
            m_pidController.SetReference(driveVelocitySetpoint, rev::CANSparkMax::ControlType::kVelocity);
        }
    }
}

Rotation2d SwerveModule::getSteerEncoder()
{
    return steerEnc.getAbsolutePosition();
}

double SwerveModule::getSteerOutput()
{
    return currentSteerOutput;
}

double SwerveModule::getDriveEncoderVel()
{
    return driveEnc.GetVelocity();
}

double SwerveModule::getDriveEncoderPos()
{
    return driveEnc.GetPosition();
}

/**
 * Set moduleInhibit to true
 * Stops motors and exits PID loop
 * Intended for disabledInit()
 *
 */
void SwerveModule::stopModule()
{
    moduleInhibit = true;
}

/**
 * Set moduleInhibit to false
 * Enter PID loop, motors are ON
 * Intended for teleop/auto init functions
 */
void SwerveModule::startModule()
{
    moduleInhibit = false;
}