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

class Shooter {
    rev::CANSparkMax *bottomMotor = new rev::CANSparkMax(bottomMotorID, rev::CANSparkMax::MotorType::kBrushless);
    rev::CANSparkMax *topMotor = new rev::CANSparkMax(topMotorID, rev::CANSparkMax::MotorType::kBrushless);
<<<<<<< HEAD
    rev::CANSparkMax *angleMotor = new rev::CANSparkMax(angleMotorID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkPIDController angleControl = angleMotor->GetPIDController();
    rev::SparkPIDController shooterControl = topMotor->GetPIDController();
    rev::SparkRelativeEncoder shooterEncoder = topMotor->GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
    rev::SparkRelativeEncoder angleEncoder = angleMotor->GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
    std::thread shooterThread;

    bool stopAngleMotor = false;
    bool stopShooterMotor = false;
    int maxRPM = 5700;
    float angleSetpoint;
    float motorVelocitySetpoint = 500;
    float shooterkP = 0, shooterkI = 0, shooterkD = 0, anglekP = 0, anglekI = 0, anglekD = 0;
=======
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
>>>>>>> 702b624de0c2d0126f2b302eb8ddfa507417e64f

    void initShooter();
    void run();
    void enableMotors();
    void stopMotors();

    void setAngleSetpoint(float setpoint);
    void setMotorVelocitySetpoint(float setpoint);
<<<<<<< HEAD
    double getShooterAngle();
    bool isFinished(float percentageBound);
=======
    
    double getAnglePosition();
    double getMotorVelocity();
>>>>>>> 702b624de0c2d0126f2b302eb8ddfa507417e64f
};