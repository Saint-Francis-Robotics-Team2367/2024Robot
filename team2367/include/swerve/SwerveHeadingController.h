#pragma once
#include <Constants.h>
#include "geometry/Pose2d.h"
#include <cmath>
#include <stdexcept>
#include <frc/controller/PIDController.h>


class SwerveHeadingController {
private:
    frc::PIDController mPIDCtr {0.02, 0.0, 0.0};
    double mSetpoint = 0.0;
        
    
    Rotation2d desiredHeading;

public:
    enum HeadingControllerState {
        OFF, SNAP, MAINTAIN
    };
    HeadingControllerState mHeadingControllerState = OFF;


    SwerveHeadingController() {
        mPIDCtr.EnableContinuousInput(0, 360);
    }

    HeadingControllerState getHeadingControllerState() {
        return mHeadingControllerState;
    }

    void setSetpoint(double inp) 
    {
        mSetpoint = inp;
    }

    void setSetpointPOV(int POV) {
        if (POV >= 0) {
            mSetpoint = POV;
        }
    }

    void setHeadingControllerState(HeadingControllerState state) {
        mHeadingControllerState = state;
    }

    double calculate(double current_angle) {
        switch (mHeadingControllerState) {
            case OFF:
                return 0.0;
            case SNAP:
                mPIDCtr.SetPID(0.02, 0.0, 0.0);
                break;
            case MAINTAIN:
                // maintain pids
                break;
        }
        return mPIDCtr.Calculate(current_angle, mSetpoint);
    }


};
