#include "PID.h"

PID::PID(double kp, double ki, double kd, double setpoint, double maxSpeed)
    : kp_(kp), ki_(ki), kd_(kd), integral_(0.0), previous_error_(0.0), setpoint_(setpoint), maxSpeed_(maxSpeed) {}

double PID::update(double measurement, double dt) {

    double error = setpoint_ - measurement;

    double P = kp_ * error;

    // Cap the stored integral so the I term can't ask for more than the
    // motor can actually give. Left alone, an integrator on a long move
    // takes as long to unwind as it took to build up.
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

    double derivative = (error - previous_error_) / dt;
    double D = kd_ * derivative;

    previous_error_ = error;

    double output = P + I + D;

    if(output > maxSpeed_) {
        output = maxSpeed_;
        integral_ -= error * dt;  // saturated, so undo this step's integral
    }else if(output < -maxSpeed_) {
        output = -maxSpeed_;
        integral_ -= error * dt;  // same going the other way
    }
    
    return output;
}