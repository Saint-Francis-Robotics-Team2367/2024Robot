#include "sensors/Limelight.h"
#include "geometry/Rotation2d.h"
#include "geometry/Translation2d.h"
#include <rev/CANSparkmax.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <thread>

#define bottomMotorID 0 //change later
#define topMotorID 1

class Shooter 
{
    rev::CANSparkMax *bottomMotor = new rev::CANSparkMax(bottomMotorID, rev::CANSparkMax::MotorType::kBrushless);
    rev::CANSparkMax *topMotor = new rev::CANSparkMax(topMotorID, rev::CANSparkMax::MotorType::kBrushless);
    std::thread motorThread;




};