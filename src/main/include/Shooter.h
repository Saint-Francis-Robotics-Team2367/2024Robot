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
#define angleMotorID 2

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
    rev::CANSparkMax *angleMotor = new rev::CANSparkMax(angleMotorID, rev::CANSparkMax::MotorType::kBrushless);
    std::thread shooterThread;
    rev::SparkPIDController angleControl;
    rev::SparkPIDController shooterControl;
    rev::SparkRelativeEncoder angleEncoder;
    rev::SparkRelativeEncoder shooterEncoder;

    bool stopAngleMotor = false;
    bool stopShooterMotor = false;
    float angleSetpoint;
    float motorVelocitySetpoint;

    void initShooter();
    void run();
    void enableMotors();
    void stopMotors();
    void setAngleSetpoint(float setpoint);
    void setMotorVelocitySetpoint(float setpoint);
    double getAnglePosition();
    double getMotorVelocity();
    bool isFinished(float percentageBound);
};