#pragma once

#define PI 3.14159265359
// #define driveEncConvFactor 6.86 * 2 * M_PI
// #define steerEncConvFactor 12.8 * 2 * M_PI

// Controller Settings
#define ctrDeadzone 0.05





// Module Constraints
#define moduleMaxFPS 16.254667 // feet per sec
#define moduleMaxRPM 5700 // RPM
#define moduleMaxRot 1.0  // Radians/sec

// Drivebase Measurements
#define trackWidth 2.375 // feet
#define wheelBase 2.375  // feet
#define moduleDriveRatio 6.12 // L3
#define wheelRadiusInches 2 // inches
#define wheelCircumFeet 1.0471976 // feet







// Util
#define kEpsilon 1e-12

class Constants
{
public:
    // Swerve Heading Controller PID
    // static const double kSnapSwerveHeadingKp = 0.05;
    // static const double kSnapSwerveHeadingKi = 0.0;
    // static const double kSnapSwerveHeadingKd = 0.0075;




};