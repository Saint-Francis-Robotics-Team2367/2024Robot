#pragma once
#include <frc/controller/PIDController.h>

class PIDControllerClass {
 public:
  PIDControllerClass(double p, double i, double d, double setpoint);

  void SetSensorValue(double sensorValue);
  double GetMotorOutput();

 private:
  frc::PIDController* pidController;
  double setpoint;
};
