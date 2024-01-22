#include "sensors/Limelight.h"
#include "geometry/Rotation2d.h"
#include "geometry/Translation2d.h"
#include <rev/CANSparkmax.h>
#include <thread>
#include <rev/SparkPIDController.h>
#include <rev/SparkRelativeEncoder.h>
#include <frc/smartdashboard/SmartDashboard.h>

#define bottomMotorID 0 //change later
#define topMotorID 1
#define tiltMotorID 2

#define revkP 6e-5 //rev basic settings
#define revkI 1e-6
#define revkFF 0.000015
#define revkMaxOutput 1.0
#define revkMinOutput -1.0
#define anglekP 0 //anglePID
#define anglekI 0
#define anglekD 0
#define shooterkP 0 //shooterPID
#define shooterkI 0
#define shooterkD 0

class Shooter {
    rev::CANSparkMax *bottomMotor = new rev::CANSparkMax(bottomMotorID, rev::CANSparkMax::MotorType::kBrushless);
    rev::CANSparkMax *topMotor = new rev::CANSparkMax(topMotorID, rev::CANSparkMax::MotorType::kBrushless);
    // Angle of shooter
    rev::CANSparkMax *tiltMotor = new rev::CANSparkMax(tiltMotorID, rev::CANSparkMax::MotorType::kBrushless);

    std::thread motorThread;

    // PID controllers
    rev::SparkPIDController tiltController;
    rev::SparkPIDController topShooterController;
    rev::SparkPIDController bottomShooterController;

    // Encoders
    rev::SparkRelativeEncoder tiltEncoder;
    rev::SparkRelativeEncoder topShooterEncoder;
    rev::SparkRelativeEncoder bottomShooterEncoder;

    double maxTiltAngle = 90;

    
    bool stopTiltMotor = true;
    bool stopShooterMotor = true;

    double tiltSetpoint;
    double shooterVelocitySetpoint;

    void initShooter();
    void run();
    void enableMotors();
    void stopMotors();

    void setAngleSetpoint(float setpoint);
    void setMotorVelocitySetpoint(float setpoint);
    
    double getAnglePosition();
    double getMotorVelocity();
};