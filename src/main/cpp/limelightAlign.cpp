#include "LimelightHelpers.h"

using namespace LimelightHelpers;


// Function to initialize Limelight (assuming LimelightHelpers.h handles this)
void InitLimelight() {
  // Perform any setup needed for LimelightHelpers
  // (LimelightHelpers.h should handle this)
}

// Function to retrieve the number of targets detected by Limelight
int GetNumTargets() {
  // Assuming LimelightHelpers provides a function to get numTargets
  return GetNumTargets();
}

// Function to retrieve the Y coordinate of a specific target
double GetTargetY(int targetID) {
  // Assuming LimelightHelpers provides a function to get targetY
  return GetTargetY(targetID);
}

// Function to process Limelight data (assuming target ID is always 4)
void ProcessLimelightData() {
  const int numTargets = GetNumTargets();
  if (numTargets > 0) {
    const double targetY = GetTargetY(4);
    const double distanceFromCenter = targetY - 11.75; // Assuming 23.5 degree vertical FOV

    // Perform actions based on distanceFromCenter (e.g., adjust aiming mechanism)

    // You can add code to send data to SmartDashboard here
  }
}
