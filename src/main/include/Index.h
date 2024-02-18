#include <rev/SparkPIDController.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkRelativeEncoder.h>


class Index{
    double speed;
    double indexP;
    double indexI;
    double indexD;
    int indexMotorID;
    int maxRPM = 5700;

    rev::SparkPIDController indexPID;
    rev::CANSparkMax indexMotor;
    rev::SparkRelativeEncoder indexEncoder = indexMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

    void enableIndex(){}
    void disableIndex(){}
    void setVelocity(double vel){}
    void setDistance(){}
};

