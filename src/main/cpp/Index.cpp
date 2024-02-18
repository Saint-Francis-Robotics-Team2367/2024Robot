#include "Index.h"

void Index::enableIndex(){
    indexPID.SetD(indexD, indexMotorID);
    indexPID.SetP(indexP, indexMotorID);
    indexPID.SetI(indexI, indexMotorID);
    setVelocity(maxRPM);
    indexPID.SetReference(speed, rev::CANSparkBase::ControlType::kVelocity);
    

    indexMotor.Set(speed/maxRPM);
    
}

void Index::disableIndex(){
    setVelocity(0);
    indexPID.SetReference(speed,rev::CANSparkBase::ControlType::kVelocity);

}

void Index::setVelocity(double vel){
    speed = vel;
}

void Index::setDistance(){

}


