#pragma once


class PID {
public:
    PID(double kp, double ki, double kd, double setpoint, double maxSpeed);
    double update(double measurement, double dt);
    void setSetpoint(double sp) { setpoint_ = sp; }

    // Throw away the integral without touching the setpoint. G1 calls this
    // when the reference reaches the end of a move: the integral built up
    // over the cruise is really just velocity feedforward, and hanging on
    // to it past the ramp is what made long moves overshoot and then hunt.
    void resetIntegral() { integral_ = 0.0; }

private:
    double kp_, ki_, kd_;
    double integral_ = 0.0;
    double previous_error_ = 0.0;
    double setpoint_;
    double maxSpeed_;
};