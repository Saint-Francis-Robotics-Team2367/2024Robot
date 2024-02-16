#pragma once

#include <vector>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>
#include <iostream>
#include "LimelightHelpers.h"
#include <algorithm>
#include "geometry/Pose3d.h"

#define PI 3.14159265359

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
            frc::SmartDashboard::PutNumber("distanceToWall", distanceToWall);
            return distanceToWall;
        }
        else
        {
            return -1.0;
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
            return NULL;
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

    /*

    void speaker() {
        LimelightHelpers::getFiducialID("limelight");//finish later
    }

    void amp() {
        LimelightHelpers::getFiducialID("limelight");//finish later
    }

    std::vector<double> getFieldPos(){
        std::vector<double> pose = LimelightHelpers::getBotpose("");
        return pose; //TX, TY, TZ, RX, RY, RZ
    }

    std::vector<double> getTargetPoseRobotSpace() {
        //std::vector<double> pose = LimelightHelpers::getTargetPose_RobotSpace("limelight");
        //Above code returned an empty vector for some reason
        std::vector<double> pose = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumberArray("targetpose_robotspace", std::span<double>{});
        return pose;
    }





inline std::vector<double> getBotpose(const std::string &limelightName = "")
{
    return getLimelightNTDoubleArray(limelightName, "botpose");
}

inline std::vector<double> getBotpose_wpiRed(const std::string &limelightName = "")
{
    return getLimelightNTDoubleArray(limelightName, "botpose_wpired");
}

inline std::vector<double> getBotpose_wpiBlue(const std::string &limelightName = "")
{
    return getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
}

inline std::vector<double> getBotpose_TargetSpace(const std::string &limelightName = "")
{
    return getLimelightNTDoubleArray(limelightName, "botpose_targetSpace");
}

inline std::vector<double> getCameraPose_TargetSpace(const std::string &limelightName = "")
{
    return getLimelightNTDoubleArray(limelightName, "camerapose_targetspace");
}

inline std::vector<double> getTargetPose_CameraSpace(const std::string &limelightName = "")
{
    return getLimelightNTDoubleArray(limelightName, "targetpose_cameraspace");
}

inline std::vector<double> getTargetPose_RobotSpace(const std::string &limelightName = "")
{
    return getLimelightNTDoubleArray(limelightName, "targetpose_robotspace");
}








    std::vector<double> getTargetAngles(double in_x, double in_y) { // Gets ax, ay
        double px = in_x;
        double py = in_y;
        // double tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0.0);
        // double tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0.0);

        // frc::SmartDashboard::PutNumber("TX", px);
        // frc::SmartDashboard::PutNumber("TY", py);

        double nx = (1/160) * (px - 159.5);
        double ny = (1/120) * (119.5 - py);

        double horizontal_fov = 54;
        double vertical_fov = 41;

        double vpw = 2.0*tan(horizontal_fov/2);
        double vph = 2.0*tan(vertical_fov/2);

        double x = vpw/2 * nx;
        double y = vph/2 * ny;

        double ax = atan2(1,x);
        double ay = atan2(1,y);

        // double axr = (ax * PI)/180;
        // double ayr = (ay * PI)/180;

        // double d = 4;
        // double rx = d * tan(axr);
        // double ry = sqrt(d*d + rx*rx) * tan(ayr);

        frc::SmartDashboard::PutNumber("AX", ax);
        frc::SmartDashboard::PutNumber("AY", ay);
        return std::vector<double> {ax, ay};
    }*/
};