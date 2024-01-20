#include "Shooter.h"

void Shooter::initShooter() {
    bottomMotor->RestoreFactoryDefaults();
    topMotor->RestoreFactoryDefaults();
    bottomMotor->SetInverted(true); //check later
    topMotor->SetInverted(false);
    shooterThread = std::thread(&run, this);

}

void Shooter::run() { 

    if (stopMotor==true) {
        bottomMotor->StopMotor();
        topMotor->StopMotor();
    }
    else {
        enableMotors();
        //bottomMotor->Set(); calculate later
        //topMotor->Set();
    }

}

void Shooter::stopMotors() {
    stopMotor = true;
}

void Shooter::enableMotors() {
    stopMotor = false;
}