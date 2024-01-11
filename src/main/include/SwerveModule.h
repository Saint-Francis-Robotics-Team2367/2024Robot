#pragma once

#include <thread>
#include <string>

#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkPIDController.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/CANSparkMax.h>

#include "sensors/CAN_Coder.h"
#include "geometry/Translation2d.h"
#include "swerve/SwerveModuleState.h"
#include "Constants.h"
#include "util/ShuffleUI.h"

// Steer PID values(custom, untuned)
constexpr float steerP = 0.76;
#define steerI 0.0
#define steerD 0.0

// Drive Velocity PID Values(Defaults from REV)
#define revkP 6e-5
#define revkI 1e-6
#define revkD 0
#define revkIz 0
#define revkFF 0.000015
#define revkMaxOutput 1.0
#define revkMinOutput -1.0


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
    float kP = revkP, kI = revkI, kD = revkD, kIz = revkIz, kFF = revkFF, kMaxOutput = revkMaxOutput, kMinOutput = revkMinOutput;

    // PID Controller for Drive Motor
    rev::SparkPIDController m_pidController;

    float driveVelocitySetpoint;
    float drivePositionSetpoint;
    float steerAngleSetpoint;
    // TODO make enum
    bool driveModePosition = false; // whether we are controlling module velocity or position
    bool moduleInhibit = false;
    const int maxRPMFreeSpeed = moduleMaxRPM;
    double currentSteerOutput = 0.0;

    // public:
    SwerveModule(int steerMotorID, int driveMotorID, int CAN_ID);
    void initMotors();

    // Getters
    float getSteerAngleSetpoint();

    // Setpoints
    void setSteerAngleSetpoint(float setpt);
    bool setSteerAngleSetpointShortestPath(float setpt);
    void setDrivePositionSetpoint(float setpt);
    void setDriveVelocitySetpoint(float setpt);
    void setDrivePercentVelocitySetpoint(float setpt);
    void setModuleState(SwerveModuleState setpt);

    // Encoders
    Rotation2d getSteerEncoder();
    double getSteerOutput();
    double getDriveEncoderVel();
    double getDriveEncoderPos();
    bool isFinished(float percentageBound);
    SwerveModuleState getModuleState();

    // Threading
    void run();
    void stopModule();
    void startModule();
};