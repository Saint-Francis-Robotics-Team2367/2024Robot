#include "SwerveModule.h"
#include <string>

SwerveModule::SwerveModule(int steerMotorID, int driveMotorID, int cancoderID) : steerMotor(new rev::CANSparkMax(steerMotorID, rev::CANSparkMax::MotorType::kBrushless)),
                                                                                 driveMotor(new rev::CANSparkMax(driveMotorID, rev::CANSparkMax::MotorType::kBrushless)),
                                                                                 steerEnc(CAN_Coder(cancoderID)),
                                                                                 driveEnc(driveMotor->GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42)),
                                                                                 m_pidController(driveMotor->GetPIDController()),
                                                                                 steerCTR(frc::PIDController(steerP, steerI, steerD))
{
    steerID = steerMotorID;
    driveID = driveMotorID;
}

void SwerveModule::initMotors()
{
    // Resetting Motor settings, Encoders, putting it in brake mode
    steerMotor->RestoreFactoryDefaults();
    driveMotor->RestoreFactoryDefaults();
    steerMotor->ClearFaults();
    driveMotor->ClearFaults();

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
    steerAngleSetpoint = 0.0;// steerEnc.getAbsolutePosition().getRadians();
    driveVelocitySetpoint = 0.0;

    // Set PID values for REV Drive PID
    m_pidController.SetP(kP);
    m_pidController.SetI(kI);
    m_pidController.SetFF(kFF);
    m_pidController.SetIAccum(0.0);
    m_pidController.SetOutputRange(kMinOutput, kMaxOutput);
    steerCTR.EnableContinuousInput(0, 2 * PI);
    steerCTR.SetIZone(steerIZone);

    // driveMotor->SetClosedLoopRampRate(0.5);

    // Clears Motor Controller's Log of errors
    // steerMotor->ClearFaults();
    // driveMotor->ClearFaults();
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

void SwerveModule::setDriveCurrentLimit(int limit)
{
    driveMotor->SetSmartCurrentLimit(limit);
}

/**
 * speedFPS attribute should be in RPM
 * Sets Drive Velocity & Steer Angle
 */
void SwerveModule::setModuleState(SwerveModuleState setpt, bool takeShortestPath)
{
    if (takeShortestPath)
    {
        SwerveModuleState outputs = moduleSetpointGenerator(getModuleState(), setpt);
        setDriveVelocitySetpoint(outputs.getSpeedFPS());
        setSteerAngleSetpoint(outputs.getRot2d().getRadians());
        prevSetpoint.setRot2d(outputs.getRot2d());
        prevSetpoint.setSpeedFPS(outputs.getSpeedFPS());
    }
    else
    {
        setDriveVelocitySetpoint(setpt.getSpeedFPS());
        setSteerAngleSetpoint(setpt.getRot2d().getRadians());
        prevSetpoint.setRot2d(setpt.getRot2d());
        prevSetpoint.setSpeedFPS(setpt.getSpeedFPS());
    }
}

SwerveModuleState SwerveModule::moduleSetpointGenerator(SwerveModuleState currState, SwerveModuleState desiredSetpoint)
{
    double currAngle = currState.getRot2d().getRadians();
    double currVel = currState.getSpeedFPS();
    double desAngle = desiredSetpoint.getRot2d().getRadians();
    double desVel = desiredSetpoint.getSpeedFPS();

    // double limitVel = ControlUtil::limitAcceleration(currVel, desVel, maxDriveAccelerationRPM, loopTime);
    // desVel = limitVel;

    double dist = fabs(currAngle - desAngle);
    bool flip = (dist > PI_2) && (((PI * 2) - dist) > PI_2);

    double setpointAngle;
    double setpointVel;
    if (flip)
    {
        setpointAngle = Rotation2d::radiansBound(desAngle + PI);

        double angleDist = std::min(fabs(setpointAngle - currAngle), (PI * 2) - fabs(setpointAngle - currAngle));

        // setpointVel = -(desVel * (-angleDist / PI_2) + desVel);
        setpointVel = -ControlUtil::scaleSwerveVelocity(desVel, angleDist, false);
        
    }
    else
    {
        setpointAngle = desAngle;

        double angleDist = std::min(fabs(setpointAngle - currAngle), (PI * 2) - fabs(setpointAngle - currAngle));

        // setpointVel = desVel * (-angleDist / PI_2) + desVel;
        setpointVel = ControlUtil::scaleSwerveVelocity(desVel, angleDist, false);
    }
    return SwerveModuleState(setpointVel, Rotation2d(setpointAngle));
}

SwerveModuleState SwerveModule::getModuleState()
{
    double vel = getDriveEncoderVel();
    double angle = steerEnc.getAbsolutePosition().getRadians();

    return SwerveModuleState(vel, angle);
}

frc::SwerveModulePosition SwerveModule::getModulePosition()
{
    frc::SwerveModulePosition pos;

    pos.angle = frc::Rotation2d(units::radian_t(Rotation2d::polarToCompass(getSteerEncoder().getRadians())));

    // changes from encoder rotations to feet to meters
    pos.distance = units::meter_t(getDriveEncoderPos() * moduleDriveRatio * wheelCircumFeet / 3.281);

    return pos;
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
    if (moduleInhibit) // Thread is in standby mode
    {

        steerMotor->StopMotor();
        driveMotor->StopMotor();
    }

    else
    {
        // Steer PID
        frc::SmartDashboard::PutNumber("SteerReading" + std::to_string(steerID), steerEnc.getAbsolutePosition().getDegrees());
        frc::SmartDashboard::PutNumber("Steerset" + std::to_string(steerID), steerAngleSetpoint * (180 / M_PI));
        double newSteerOutput = steerCTR.Calculate(steerEnc.getAbsolutePosition().getRadians(), steerAngleSetpoint);
        if (currentSteerOutput != newSteerOutput) // Save some CAN buffer
        {
            currentSteerOutput = newSteerOutput;
            steerMotor->Set(currentSteerOutput);
        }
        
        frc::SmartDashboard::PutNumber("Driveset" + std::to_string(steerID), driveVelocitySetpoint);

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

/**
 * in rotations
 */
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