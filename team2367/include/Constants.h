#pragma once

#define PI 3.14159265359
#define PI_2 3.14159265359/2
// #define driveEncConvFactor 6.86 * 2 * M_PI
// #define steerEncConvFactor 12.8 * 2 * M_PI

// Controller Settings
#define ctrDeadzone 0.09
#define ctrSlewRate 0.1

// Module Constraints
#define moduleMaxFPS 16.254667 // feet per sec
#define moduleMaxRPM 5700      // RPM
#define moduleMaxRot 2.0 // 9.678 / 2, Radians/sec

// Drivebase Measurements
#define trackWidth 2.375          // feet
#define wheelBase 2.375           // feet
#define moduleDriveRatio 6.12     // L3
#define wheelRadiusInches 2       // inches
#define wheelCircumFeet 1.0471976 // feet

// Util
#define kEpsilon 1e-12
#define loopTime 0.02

namespace motorIDs 
{
// Arm
constexpr int leftFrontMotorID = 9;
constexpr int leftBackMotorID = 11;
constexpr int rightFrontMotorID = 10;
constexpr int rightBackMotorID = 12;

// Intake & Index
constexpr int indexMotorID = 16;
constexpr unsigned int intakeMotorID = 19;

// Shooter
constexpr int bottomRollerID = 14;
constexpr int topRollerID = 15;

};

class Constants
{
public:
};