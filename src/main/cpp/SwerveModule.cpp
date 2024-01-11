#include "SwerveModule.h"

SwerveModule::SwerveModule(int steerMotorID, int driveMotorID, int CANencoderID) : steerMotor(new rev::CANSparkMax(steerMotorID, rev::CANSparkMax::MotorType::kBrushless)),
                                                                             driveMotor(new rev::CANSparkMax(driveMotorID, rev::CANSparkMax::MotorType::kBrushless)),
                                                                             steerEnc(CAN_Coder(CANencoderID)),
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

    // No inverts needed due to CANCoder
    steerMotor->SetInverted(true);
    driveMotor->SetInverted(true);

    // To be changed to absolute position
    steerEnc.encoder.SetPosition(steerEnc.getAbsolutePosition().getDegrees());
    driveEnc.SetPosition(0);

    // Makes motor stiff(coast mode lets it run freely)
    steerMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    driveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    // Keep the motor limit at under 20A
    steerMotor->SetSmartCurrentLimit(20);
    driveMotor->SetSmartCurrentLimit(20);

    // Conversion factor from Rotations of motor, which is nothing for now
    driveEnc.SetPositionConversionFactor(1.0);

    // Setpoints to initial encoder positions/speeds
    steerAngleSetpoint = steerEnc.getPosition().getRadians();
    driveVelocitySetpoint = 0.0;

    // Set PID values for REV Drive PID
    m_pidController.SetP(kP);
    m_pidController.SetI(kI);
    m_pidController.SetD(kD);
    m_pidController.SetIZone(kIz);
    m_pidController.SetFF(kFF);
    m_pidController.SetOutputRange(kMinOutput, kMaxOutput);

    steerCTR.EnableContinuousInput(0, 2 * M_PI);
    steerMotor->ClearFaults();
    driveMotor->ClearFaults();
    // m_pidController.SetSmartMotionAccelStrategy(rev::CANPIDController::AccelStrategy::kSCurve);
    //m_pidController.SetSmartMotionMaxAccel()
}

float SwerveModule::getSteerAngleSetpoint()
{
    return steerAngleSetpoint;
}

/**
 * Enter in radians
 * Will modulus it to 0 - 2pi
 */
void SwerveModule::setSteerAngleSetpoint(float setpt)
{
    steerAngleSetpoint = Rotation2d::radiansBound(setpt);
}

/* Takes in input from 0 - 2pi
 * 0 is the right, goes counterclockwise
 * Not tested
 */
bool SwerveModule::setSteerAngleSetpointShortestPath(float setpt)
{
    double currAngle = Rotation2d::radiansBound(steerEnc.getPosition().getRadians());
    double setAngle = Rotation2d::radiansBound(setpt);
    bool flip = false;
    if (fabs(currAngle - setAngle) > (M_PI / 2)) // Flipping drive direction = shorter
    {
        frc::SmartDashboard::PutBoolean("Flip", true);
        flip = true;
        setSteerAngleSetpoint(setAngle - (M_PI));
    } else 
    {
        setSteerAngleSetpoint(setAngle);
        frc::SmartDashboard::PutBoolean("Flip", false);
        
    }
    return flip;
}

/**
 * Untested, I've never used it
 */
void SwerveModule::setDrivePositionSetpoint(float setpt)
{
    drivePositionSetpoint = setpt;
    driveModePosition = true;
}

/**
 * Set the drive motor velocity setpoint to input RPM
 * Max RPM is 5700
 */
void SwerveModule::setDriveVelocitySetpoint(float setpt)
{
    driveVelocitySetpoint = setpt;
    driveModePosition = false;
}

/**
 * Set the drive motor velocity setpoint to the input percent of max RPM
 *  Input shld be in [-1, 1]
 */
void SwerveModule::setDrivePercentVelocitySetpoint(float setpt)
{
    setDriveVelocitySetpoint(maxRPMFreeSpeed * setpt);
}

/**
 * speedMPS attribute should be in RPM
 * Sets Drive Velocity & Steer Angle
 */
void SwerveModule::setModuleState(SwerveModuleState setpt)
{
    bool flip = setSteerAngleSetpointShortestPath(setpt.getRot2d().getRadians());
    if (flip) {
        setDriveVelocitySetpoint(-setpt.getSpeedFPS());
    } else {
        setDriveVelocitySetpoint(setpt.getSpeedFPS());
    }
    // setSteerAngleSetpoint(setpt.getRot2d().getRadians());
    // setDriveVelocitySetpoint(setpt.getSpeedFPS());
}

SwerveModuleState SwerveModule::getModuleState() {
    double vel = getDriveEncoderVel();
    // Gyro widget is in compass format, encoders r in polar
    double angle = Rotation2d::polarToCompass(getSteerEncoder().getDegrees());

    
    if (vel < 0) {
        angle = angle + 180;
    }
    angle = Rotation2d::degreesBound(angle);
    // ShuffleUI::MakeWidget(name, driveTab, angle, frc::BuiltInWidgets::kGyro, row, col);
    return SwerveModuleState(fabs(vel), angle * M_PI / 180);
}

/**
 * Unfinished, Untested, pending review
 */
bool SwerveModule::isFinished(float percentageBound)
{
    if (driveModePosition)
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

    if (moduleInhibit) // Thread is in standby mode
    {

        steerMotor->StopMotor();
        driveMotor->StopMotor();
    }

    else
    {
        // Steer PID
        double steerOutput = steerCTR.Calculate(steerEnc.getAbsolutePosition().getRadians(), steerAngleSetpoint);
        currentSteerOutput = steerOutput;
        steerMotor->Set(steerOutput);

        // Drive Motor uses the internal REV PID, since optimizations here are rarely needed
        if (driveModePosition)
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

double SwerveModule::getSteerOutput() {
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