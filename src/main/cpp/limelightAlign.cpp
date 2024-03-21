//Google.com/Gemini Utilised AI to Code
#include "sensors/Limelight.h"
#include "SwerveDrive.h"

//moves robot horizontally to centre with limelight (still have to add offset)
double horizontalMovementAmp(Limelight limelight){
    if (limelight.isAmpTagDetected()){ //checks if amp tag is visible
        double tx = limelight.getAngleLimelightToWall();
        double distance = sin(tx)*limelight.getDistanceToWall(); //calculates distance from crosshair to tag
        return distance;
    }
}

void llAutoRotation(){  //rotates robot towards april tag
    Pose3d target = mLimelight.getTargetPoseRobotSpace(); //pulled code from old function
      double angleOffset = Rotation2d::polarToCompass(atan2(target.y, target.x)) * 180 / PI;
      double zeroSetpoint = mGyro.getBoundedAngleCW().getDegrees() + angleOffset;
      mHeadingController.setHeadingControllerState(SwerveHeadingController::ALIGN);
      mHeadingController.setSetpoint(zeroSetpoint);
}

bool autoShoot(Limelight limelight){ //checks how far robot is from wall
    if (limelight.getDistanceToWall <= .3){ //makes sure that robot is less than ~1ft from wall
        return(true); //returns true to auto unload shooter
    }
}