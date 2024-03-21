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

void rotation(){
    Pose3d target = mLimelight.getTargetPoseRobotSpace();
      double angleOffset = Rotation2d::polarToCompass(atan2(target.y, target.x)) * 180 / PI;
      double zeroSetpoint = mGyro.getBoundedAngleCW().getDegrees() + angleOffset;
      mHeadingController.setHeadingControllerState(SwerveHeadingController::ALIGN);
      mHeadingController.setSetpoint(zeroSetpoint);
}

bool autoShoot(Limelight limelight){
    if (limelight.getDistanceToWall <= .3){
        return(true);
    }
}