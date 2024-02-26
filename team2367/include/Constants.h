#pragma once

#define PI 3.14159265359
#define PI_2 1.570796326

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

constexpr float velocityP = 6e-5;
constexpr float velocityI = 1e-6;
constexpr float velocityD = 0.0;
constexpr float velocityFF = 0.000015;

constexpr float positionP = 0.06;
constexpr float positionI = 0.0;
constexpr float positionD = 0.0;
constexpr float positionFF = 0.0;

namespace motorIDs 
{
// Arm
constexpr int leftFrontMotorID = 19;
constexpr int leftBackMotorID = 11;
constexpr int rightFrontMotorID = 10;
constexpr int rightBackMotorID = 12;

// Intake & Index
constexpr int indexMotorID = 16;
constexpr int intakeMotorID = 17;

// Shooter
constexpr int bottomRollerID = 14;
constexpr int topRollerID = 15;

};

class Constants
{
public:
};