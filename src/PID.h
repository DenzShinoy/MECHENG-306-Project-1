#pragma once


class PID {
public:
    PID(double kp, double ki, double kd, double setpoint, double maxSpeed);
    double update(double measurement, double dt);
    double getPreviousError() const { return previous_error_; }
    void setSetpoint(double sp) { setpoint_ = sp; }

private:
    double kp_, ki_, kd_;
    double integral_ = 0.0;
    double previous_error_ = 0.0;
    double setpoint_;
    double maxSpeed_;
};