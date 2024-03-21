//Google.com/Gemini Utilised AI to Code
#include "sensors/Limelight.h"
#include "SwerveDrive.h"

void horizontalMovementAmp(SwerveDrive mDrive, Limelight limelight){
    if (limelight.isAmpTagDetected()){
        double tx = limelight.getAngleLimelightToWall();
        double distance = sin(tx)*limelight.getDistanceToWall();
        mDrive.autoMove(0, distance);
    }
}