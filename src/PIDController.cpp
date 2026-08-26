#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd,
                            float outMin, float outMax)
    : _kp(kp), _ki(ki), _kd(kd), _outMin(outMin), _outMax(outMax),
      _integral(0.0f), _prevError(0.0f) {}

float PIDController::compute(float setpoint, float measurement, float dt) {
  // TODO: error = setpoint - measurement; P + I(dt) + D(dt);
  // clamp output to [_outMin, _outMax] with integral anti-windup.
  (void)setpoint; (void)measurement; (void)dt;
  return 0.0f;
}

void PIDController::reset() {
  // TODO: _integral = 0; _prevError = 0.
}

void PIDController::setGains(float kp, float ki, float kd) {
  _kp = kp; _ki = ki; _kd = kd;
}
