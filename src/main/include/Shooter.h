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

constexpr float shooterP = 6e-5;
constexpr float shooterI = 1e-6;
constexpr float shooterD = 0.0;
constexpr float shooterFF = 0.000015;

constexpr float shooterCurrentLimit = 20;


class Shooter
{
private:
    rev::CANSparkMax topRollerMotor = rev::CANSparkMax(topRollerID, rev::CANSparkBase::MotorType::kBrushless);
    rev::CANSparkMax bottomRollerMotor = rev::CANSparkMax(bottomRollerID, rev::CANSparkBase::MotorType::kBrushless);

    rev::SparkPIDController topRollerController = topRollerMotor.GetPIDController();
    rev::SparkPIDController bottomRollerController = bottomRollerMotor.GetPIDController();

    rev::SparkRelativeEncoder topRollerEncoder = topRollerMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
    rev::SparkRelativeEncoder bottomRollerEncoder = bottomRollerMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);

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
