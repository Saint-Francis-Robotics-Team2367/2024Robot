#include <rev/SparkPIDController.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkRelativeEncoder.h>

constexpr int indexMotorID = 10;
constexpr int indexCurrentLimit = 10;

class Index
{
    float distanceSetpoint;
    float velocitySetpoint;

    bool inDistanceMode = false;
    rev::CANSparkMax indexMotor = rev::CANSparkMax(indexMotorID, rev::CANSparkLowLevel::MotorType::kBrushless);
    rev::SparkPIDController indexController = indexMotor.GetPIDController();
    rev::SparkRelativeEncoder indexEncoder = indexMotor.GetEncoder();

    void init();
    void disable();
    void setVelocity(double velocity);
    void setDistance(double distance);
    bool isDistanceFinished(float percentageBound);
};
