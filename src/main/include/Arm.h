#pragma once

#include <rev/CANSparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DutyCycleEncoder.h>
#include "geometry/Rotation2d.h"
#include <frc/controller/PIDController.h>
#include <rev/SparkPIDController.h>
#include <rev/SparkRelativeEncoder.h>
#include <thread>

#define leftFrontMotorID 9
#define leftBackMotorID 11
#define rightFrontMotorID 10
#define rightBackMotorID 12

#define revkP 6e-5
#define revkI 1e-6
#define revkD 0.0
#define revkFF 0.000015

#define bottomRollerID 14
#define topRollerID 15

#define tiltControllerP 0.02
#define tiltControllerI 0.0
#define tiltControllerD 0.0

#define armCurrentLimit 20
#define shooterCurrentLimit 20

constexpr float encoderToArmRatio = 12.0 / 54.0;
constexpr float armMinFromVertical = 84.0;
constexpr float shooterToArmAngle = 46.0;

class Arm 
{
private:
    rev::CANSparkMax leftSideLead = rev::CANSparkMax(leftFrontMotorID, rev::CANSparkBase::MotorType::kBrushless);
    rev::CANSparkMax leftSideFollow = rev::CANSparkMax(leftBackMotorID, rev::CANSparkBase::MotorType::kBrushless);
    rev::CANSparkMax rightSideLead = rev::CANSparkMax(rightFrontMotorID, rev::CANSparkBase::MotorType::kBrushless);
    rev::CANSparkMax rightSideFollow = rev::CANSparkMax(rightBackMotorID, rev::CANSparkBase::MotorType::kBrushless);

    frc::DutyCycleEncoder tiltEncoder = frc::DutyCycleEncoder(0);
    frc::PIDController tiltController{tiltControllerP, tiltControllerI, tiltControllerD};

    bool stopTiltMotor = true;
    double tiltSetpoint;
    float maxTiltSetpoint = 61;

    std::thread motorThread;
    void setAllMotors(double input);


public:
    enum armPosition {
        HIGH, STOW
    };
    void init();
    void run();
    void enableMotors();
    void disableMotors();

    Rotation2d getAxleAngle();
    Rotation2d getShooterAngle();
    void setAngleSetpoint(float setpoint);
    void setAngleSetpoint(armPosition desiredPosition);


};