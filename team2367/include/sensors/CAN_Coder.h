#pragma once


#include <ctre/Phoenix.h>
#include "geometry/Rotation2d.h"


class CAN_Coder {
    private:
    int ID;

    public:
    WPI_CANCoder encoder;
    CAN_Coder(int canID) : encoder(WPI_CANCoder(canID, "rio"))
    {
        ID = canID;
    }



    Rotation2d getPosition() {
        return Rotation2d(encoder.GetPosition() * (M_PI / 180));
    }

    Rotation2d getAbsolutePosition() {
        return Rotation2d(encoder.GetAbsolutePosition() * (M_PI / 180));
    }

    double getVelocity() {
        return encoder.GetVelocity();
    }
    int getCANID() {
        return ID;
    }

    
    

};