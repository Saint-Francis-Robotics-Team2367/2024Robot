#pragma once

#define PI 3.14159265359
// #define driveEncConvFactor 6.86 * 2 * M_PI
// #define steerEncConvFactor 12.8 * 2 * M_PI

// Controller Settings
#define ctrDeadzone 0.05



// Motor/CAN IDs
#define FLsteerID 11
#define FLdriveID 18
#define FL_CAN_ID 3 // updated

#define FRsteerID 4
#define FRdriveID 16
#define FR_CAN_ID 1 // updated

#define BLsteerID 12
#define BLdriveID 10
#define BL_CAN_ID 2 // updated

#define BRsteerID 1
#define BRdriveID 42
#define BR_CAN_ID 0 // updated

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



// Steer PID values(custom, untuned)
#define steerP 0.76
#define steerI 0.0
#define steerD 0.0

// Drive Velocity PID Values(Defaults from REV)
#define revkP 6e-5
#define revkI 1e-6
#define revkD 0
#define revkIz 0
#define revkFF 0.000015
#define revkMaxOutput 1.0
#define revkMinOutput -1.0

//Shuffleboard Tabs: THESE DONT WORK
#define driveTab "Drive"
#define ctrTab "Controller"


// Util
#define kEpsilon 1e-12

class Constants
{




};