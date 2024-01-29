#pragma once

#include <thread>
#include <string>

#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkPIDController.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/CANSparkMax.h>
#include <frc/Timer.h>

#include "sensors/CAN_Coder.h"
#include "geometry/Translation2d.h"
#include "swerve/SwerveModuleState.h"
#include "Constants.h"
#include "util/ShuffleUI.h"
#include "util/ControlUtil.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/geometry/Rotation2d.h"

// Steer PID values(custom, untuned)
constexpr float steerP = 0.60; // prev 0.76
constexpr float steerI = 0.0;
constexpr float steerD = 0.0;

// Drive Velocity PID Values(Defaults from REV)
constexpr float revkP = 6e-5;
constexpr float revkI = 1e-6;
constexpr float revkFF = 0.000015;
constexpr float revkMaxOutput = 1.0;
constexpr float revkMinOutput = -1.0;

class SwerveModule
{
    // private: <- removed for testing
public:
    int steerID;
    int driveID;

    rev::CANSparkMax *steerMotor;
    rev::CANSparkMax *driveMotor;

    CAN_Coder steerEnc;
    rev::SparkRelativeEncoder driveEnc;

    // PID Controller for Steer Motor
    frc::PIDController steerCTR{steerP, steerI, steerD};

    // REV Default Velocity PID values(Drive Motor)
    float kP = revkP, kI = revkI, kFF = revkFF, kMaxOutput = revkMaxOutput, kMinOutput = revkMinOutput;

    // PID Controller for Drive Motor
    rev::SparkPIDController m_pidController;

    // Timer for acceleration limiting
    frc::Timer aTimer = frc::Timer();

    float driveVelocitySetpoint;
    float drivePositionSetpoint;
    float steerAngleSetpoint;

    enum driveModeType
    {
        POSITION,
        VELOCITY
    };
    driveModeType driveMode = VELOCITY; // whether we are controlling module velocity or position
    bool moduleInhibit = true;         // True to stop the motors

    // Module Constraints
    const int maxRPMFreeSpeed = moduleMaxRPM;
    const float maxDriveAccelerationFPS = 7.603; // Feet per sec2
    const float maxDriveAccelerationRPM = 2665.993;
    const float maxSteerVelocity = 189.2; // Radians per sec

    const int maxSteerCurrent = 20; // Maximum current to steer motor
    const int maxDriveCurrent = 20; // Maximum current to steer motor

    // TODO: Brownout module
    double currentSteerOutput = 0.0;

    // public:
    SwerveModule(int steerMotorID, int driveMotorID, int cancoderID);
    void initMotors();

    // Getters
    float getSteerAngleSetpoint();

    // Setpoints
    void setSteerAngleSetpoint(float setpt);

    void setDrivePositionSetpoint(float setpt);
    void setDriveVelocitySetpoint(float setpt);

    // TODO: Test this
    void setModuleState(SwerveModuleState setpt, bool takeShortestPath = true);
    SwerveModuleState moduleSetpointGenerator(SwerveModuleState prevSetpoint, SwerveModuleState desiredSetpoint);
    frc::SwerveModulePosition getModulePosition(); 

    // Encoders
    Rotation2d getSteerEncoder();
    double getSteerOutput();
    double getDriveEncoderVel();
    double getDriveEncoderPos();

    // TODO: implement & overload
    bool isFinished(float percentageBound);
    SwerveModuleState getModuleState();

    // Module state control
    void run();
    void stopModule();
    void startModule();
};