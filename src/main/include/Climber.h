#include <rev/CANSparkmax.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <thread>

#define rightClimberMotorID 0 //change later
#define leftClimberMotorID 0 
class Climber {
    double minHeight; 
    double maxHeight;
    double desiredLeftHeight;
    double desiredRightHeight;
    rev::CANSparkMax *leftClimberMotor = new rev::CANSparkMax(rightClimberMotorID, rev::CANSparkMax::MotorType::kBrushless);
    rev::CANSparkMax *rightClimberMotor = new rev::CANSparkMax(leftClimberMotorID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkPIDController leftController = leftClimberMotor->GetPIDController();
    rev::SparkPIDController rightController = rightClimberMotor->GetPIDController();
    rev::SparkMaxRelativeEncoder leftEnc = leftClimberMotor->GetEncoder();
    rev::SparkMaxRelativeEncoder rightEnc = rightClimberMotor->GetEncoder();
    void moveManual(double leftInput, double rightInput);
    void init();
    void moveToMaxHeight();
    void moveToMinHeight();
    bool isAtSetpoint();
}