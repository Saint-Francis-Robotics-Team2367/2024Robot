#include <frc/controller/PIDController.h>

// Global variables for PID controller and sensor values
frc::PIDController pidController {0.0, 0.0, 0.0}; // Initialize with placeholder gains
double currentValueB = 0.0;
double desiredValueB = 0.0;

// Function to initialize the PID controller with appropriate gains
void initializePID(double Kp, double Ki, double Kd) {
    pidController = frc::PIDController(Kp, Ki, Kd);
}

// Function to set the desired value for value B
void setDesiredValueB(double newValue) {
    desiredValueB = newValue;
}

// Function to update the current value of value B
void updateCurrentValueB(double newValue) {
    currentValueB = newValue;
}

// Function to calculate the motor output value using PID control
double calculateMotorOutput() {
    double error = desiredValueB - currentValueB;
    double motorOutput = pidController.Calculate(error);

    // You may want to add output limiting or clamping here for safety

    return motorOutput;
}

// Example usage in a periodic control loop:
void controlLoop() {
    double motorOutput = calculateMotorOutput();
    // Use the motorOutput value to set your motor speed or other control actions
}
