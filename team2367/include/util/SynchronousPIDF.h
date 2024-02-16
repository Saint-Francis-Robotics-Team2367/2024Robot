#include <cmath>
#include <stdexcept>
#include <limits>
#include <chrono>
#include <string>

class SynchronousPIDF {
private:
    double m_P; // factor for "proportional" control
    double m_I; // factor for "integral" control
    double m_D; // factor for "derivative" control
    double m_F; // factor for feed forward gain
    double m_maximumOutput = 1.0; // |maximum output|
    double m_minimumOutput = -1.0; // |minimum output|
    double m_maximumInput = 0.0; // maximum input - limit setpoint to this
    double m_minimumInput = 0.0; // minimum input - limit setpoint to this
    bool m_continuous = false; // do the endpoints wrap around? eg. absolute encoder
    double m_prevError = 0.0; // the prior sensor input (used to compute velocity)
    double m_totalError = 0.0; // the sum of the errors for use in the integral calc
    double m_setpoint = 0.0;
    double m_error = 0.0;
    double m_result = 0.0;
    double m_last_input = std::numeric_limits<double>::quiet_NaN();
    double m_deadband = 0.0; // If the absolute error is less than deadband then treat error for the proportional term as 0
    double m_last_timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now().time_since_epoch()).count();

public:
    SynchronousPIDF() {}

    void setPID(double p, double i, double d) {
        m_P = p;
        m_I = i;
        m_D = d;
    }

    void setPIDF(double p, double i, double d, double f) {
        setPID(p, i, d);
        m_F = f;
    }

    double calculate(double input) {
        double timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now().time_since_epoch()).count();
        double dt = timestamp - m_last_timestamp;
        m_last_timestamp = timestamp;

        return calculate(input, dt);
    }

    double calculate(double input, double dt) {
        if (dt < 1E-6) {
            dt = 1E-6;
        }

        m_last_input = input;
        m_error = m_setpoint - input;
        if (m_continuous) {
            if (std::abs(m_error) > (m_maximumInput - m_minimumInput) / 2) {
                if (m_error > 0) {
                    m_error = m_error - m_maximumInput + m_minimumInput;
                } else {
                    m_error = m_error + m_maximumInput - m_minimumInput;
                }
            }
        }

        if (std::abs(m_error * m_P) >= m_minimumOutput && std::abs(m_error * m_P) <= m_maximumOutput) {
            m_totalError += m_error * dt;
        } else {
            m_totalError = 0;
        }

        double proportionalError = std::abs(m_error) < m_deadband ? 0 : m_error;

        m_result = (m_P * proportionalError + m_I * m_totalError + m_D * (m_error - m_prevError) / dt
                + m_F * m_setpoint);
        m_prevError = m_error;

        m_result = std::min(std::max(m_result, m_minimumOutput), m_maximumOutput);
        return std::min(m_maximumOutput, std::max(m_minimumOutput, m_result));
    }
    double getP() {
        return m_P;
    }
    double getI() {
        return m_I;
    }
    double getD() {
        return m_D;
    }
    double getF() {
        return m_F;
    }
    double get() {
        return m_result;
    }
    void setContinuous(bool continuous) {
        m_continuous = continuous;
    }
    void setDeadband(double deadband) {
        m_deadband = deadband;
    }
    void setContinuous() {
        this->setContinuous(true);
    }
    void setInputRange(double minimumInput, double maximumInput) {
        m_minimumInput = minimumInput;
        m_maximumInput = maximumInput;
        setSetpoint(m_setpoint);
    }
    void setOutputRange(double minimumOutput, double maximumOutput) {
        m_minimumOutput = minimumOutput;
        m_maximumOutput = maximumOutput;
    }
    void setMaxAbsoluteOutput(double maxAbsoluteOutput) {
        setOutputRange(-maxAbsoluteOutput, maxAbsoluteOutput);
    }
    void setSetpoint(double setpoint) {
        if (m_maximumInput > m_minimumInput) {
            if (setpoint > m_maximumInput) {
                m_setpoint = m_maximumInput;
            } else if (setpoint < m_minimumInput) {
                m_setpoint = m_minimumInput;
            } else {
                m_setpoint = setpoint;
            }
        } else {
            m_setpoint = setpoint;
        }
    }
    double getSetpoint() {
        return m_setpoint;
    }
    double getError() {
        return m_error;
    }
    // bool onTarget(double tolerance) {
    //     return !isnan(m_last_input) && abs(m_last_input - m_setpoint) < tolerance;
    // }
    void reset() {
        m_last_input = NAN;
        m_prevError = 0;
        m_totalError = 0;
        m_result = 0;
        m_setpoint = 0;
    }
    void resetIntegrator() {
        m_totalError = 0;
    }
    std::string getState() {
        std::string lState = "";

        lState += "Kp: " + std::to_string(m_P) + "\n";
        lState += "Ki: " + std::to_string(m_I) + "\n";
        lState += "Kd: " + std::to_string(m_D) + "\n";

        return lState;
    }

    std::string getType() {
        return "PIDController";
    }
};
