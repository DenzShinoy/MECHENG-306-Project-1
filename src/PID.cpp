#include "PID.h"

PID::PID(double kp, double ki, double kd, double setpoint, double maxSpeed)
    : kp_(kp), ki_(ki), kd_(kd), integral_(0.0), previous_error_(0.0), setpoint_(setpoint), maxSpeed_(maxSpeed) {}

double PID::update(double measurement, double dt) {

    double error = setpoint_ - measurement;

    // Proportional term
    double P = kp_ * error;

    // Integral term. Bound the stored integral so the I term can never
    // demand more than the actuator can give: an unbounded integrator on a
    // long move takes just as long to unwind as it took to build.
    integral_ += error * dt;

    if (ki_ > 0.0) {
        const double integralLimit = maxSpeed_ / ki_;

        if (integral_ > integralLimit) {
            integral_ = integralLimit;
        } else if (integral_ < -integralLimit) {
            integral_ = -integralLimit;
        }
    }

    double I = ki_ * integral_;

    // Derivative term
    double derivative = (error - previous_error_) / dt;
    double D = kd_ * derivative;

    // Update previous error for next iteration
    previous_error_ = error;

    double output = P + I + D;

    if(output > maxSpeed_) {
        output = maxSpeed_;
        integral_ -= error * dt; // Prevent integral windup
    }else if(output < -maxSpeed_) {
        output = -maxSpeed_;
        integral_ -= error * dt; // Prevent integral windup
    }
    
    return output;
}