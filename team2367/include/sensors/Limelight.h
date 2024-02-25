#pragma once

#include <vector>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>
#include <iostream>
#include "LimelightHelpers.h"
#include <algorithm>
#include "geometry/Pose3d.h"
#include "Constants.h"

class Limelight
{

    // TODO:
    // 1. USE PID to get in front of tag by set distance
    // 2. make fucntion for target area
    // 3. getFiducialID() use for speaker and amp

private:
    // update field and robot measurements here / talk to CAD team
    std::vector<int> speakerCenterIDs = {4, 7};
    std::vector<int> speakerSideIDs = {3, 8};
    std::vector<int> ampIDs = {5, 6};
    std::vector<int> sourceIDs = {1, 2, 9, 10};
    double limelightAngleDegrees = 36;
    double limelightHeightInches = 5;
    double tagHeightInches = 52;

public:
    bool targetDetected()
    {
        if ((nt::NetworkTableInstance::GetDefault().GetTable("limelight-lralt")->GetNumber("tv", 0)) == 0)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    double getDistanceToWall()
    {
        if (targetDetected() == true)
        {
            double ty = LimelightHelpers::getTY("");
            double angleToTagDegrees = limelightAngleDegrees + ty;
            double angleToTagRadians = angleToTagDegrees * (PI / 180.0);
            double distanceToWall = (tagHeightInches - limelightHeightInches) / tan(angleToTagRadians);
            distanceToWall = distanceToWall + 0.4191; //added the distance between limelight and shooter
            frc::SmartDashboard::PutNumber("distanceToWall", distanceToWall);
            return distanceToWall;
        }
        else {
            return 0;
        }
    }

    double getAngleLimelightToTag()
    { // TY + limelight mount angle
        if (targetDetected() == true)
        {
            double ty = LimelightHelpers::getTY("");
            double angleToTagDegrees = limelightAngleDegrees + ty;
            frc::SmartDashboard::PutNumber("vertical angle", angleToTagDegrees);
            return angleToTagDegrees;
        }
        else
        {
            return 0;
        }
    }

    double getAngleLimelightToWall()
    { // TX
        if (targetDetected() == true)
        {
            double tx = LimelightHelpers::getTX("");
            frc::SmartDashboard::PutNumber("Tx", tx);
            return tx;
        }
    }

    std::vector<double> getPolarCoords()
    {
        std::vector<double> polarCoords = {getAngleLimelightToWall(), getDistanceToWall()};
        return polarCoords;
    }

    std::vector<double> getXYCoords()
    {
        double angle = getAngleLimelightToWall() * (PI / 180);
        double x = (getDistanceToWall()) * sin(angle);
        double y = (getDistanceToWall()) * cos(angle);
        std::vector<double> xyCoords = {x, y};
        return xyCoords;
    }

    bool isIn(int object, std::vector<int> inp)
    {
        for (unsigned int i = 0; i < inp.size(); i++)
        {
            if (inp[i] == object)
            {
                return true;
            }
        }
        return false;
    }

    Pose3d getTargetPoseRobotSpace()
    {
        if (targetDetected() == true)
        {
            std::vector<double> x = LimelightHelpers::getTargetPose_RobotSpace();
            Pose3d output = Pose3d(x);
            double tempY = output.y;
            output.y = output.z;
            output.z = -tempY;
            return output;
        }
    }

    bool isSpeakerTagDetected()
    {
        return isIn((int)LimelightHelpers::getFiducialID(), speakerCenterIDs);
    }
};