#pragma once

#include "swerve/SwerveModuleState.h"
#include "swerve/ChassisSpeeds.h"
#include <vector>


class SwerveSetpoint 
{
public:
    std::vector<SwerveModuleState> modStates;
    ChassisSpeeds botSpeed;

    SwerveSetpoint(std::vector<SwerveModuleState> states, ChassisSpeeds speeds) 
    {
        for (int i = 0; i < 4; i++) {
            modStates.push_back(states[i]);
        }
        botSpeed = speeds;
    }


};