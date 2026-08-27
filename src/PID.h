#pragma once


class PID {
public:
    PID(double kp, double ki, double kd, double setpoint, double maxSpeed);
    double update(double measurement, double dt);
    void setSetpoint(double sp) { setpoint_ = sp; }

    // Drop the accumulated integral without disturbing the setpoint. Used
    // when a move's reference arrives: the integral built up during cruise
    // is velocity feedforward, and holding it past the end of the ramp is
    // what makes a long move overshoot and then hunt.
    void resetIntegral() { integral_ = 0.0; }

private:
    double kp_, ki_, kd_;
    double integral_ = 0.0;
    double previous_error_ = 0.0;
    double setpoint_;
    double maxSpeed_;
};