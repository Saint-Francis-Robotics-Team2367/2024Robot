#include "sensors/Limelight.h"
#include "geometry/Rotation2d.h"
#include "geometry/Translation2d.h"
#include <rev/CANSparkMax.h>
#include <thread>
#include <rev/SparkPIDController.h>
#include <rev/SparkRelativeEncoder.h>
#include <frc/smartdashboard/SmartDashboard.h>

constexpr int bottomRollerID = 14;
constexpr int topRollerID = 15;

constexpr float revkP = 6e-5;
constexpr float revkI = 1e-6;
constexpr float revkD = 0.0;
constexpr float revkFF = 0.000015;

constexpr float shooterCurrentLimit = 20;

class Shooter
{
private:
    rev::CANSparkMax topRollerMotor = rev::CANSparkMax(topRollerID, rev::CANSparkBase::MotorType::kBrushless);
    rev::CANSparkMax bottomRollerMotor = rev::CANSparkMax(bottomRollerID, rev::CANSparkBase::MotorType::kBrushless);

    rev::SparkPIDController topRollerController = topRollerMotor.GetPIDController();
    rev::SparkPIDController bottomRollerController = bottomRollerMotor.GetPIDController();

    rev::SparkRelativeEncoder topRollerEncoder = topRollerMotor.GetEncoder();
    rev::SparkRelativeEncoder bottomRollerEncoder = bottomRollerMotor.GetEncoder();

    const float maxVelocitySetpoint = 5700.0;
    const float lowVelocitySetpoint = 200.0;
    float distanceSetpoint;

public:
    enum shooterSpeeds
    {
        HIGH,
        LOW,
        STOP
    };

    void init();
    void disable();
    void setSpeed(shooterSpeeds speed);
    void setSpeed(float rotationsPerMinute);
    void setDistance(float distance);
    bool isDistanceFinished(float percentageBound);
};
