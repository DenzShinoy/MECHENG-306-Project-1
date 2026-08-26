#pragma once
#include <Arduino.h>

// =====================================================================
//  Module 5 — PIDController
// ---------------------------------------------------------------------
//  Generic position PID, our own code. Stateless of the machine: it only
//  knows setpoint, measurement and dt. One instance per axis. Output is
//  clamped to [_outMin, _outMax] with integral anti-windup so the term
//  does not accumulate while saturated. Non-blocking; dt is supplied by
//  the caller (seconds).
//
//  Units: works in whatever the caller uses (we feed it COUNTS); output
//  is a PWM-range command.
// =====================================================================

class PIDController {
public:
  PIDController(float kp, float ki, float kd, float outMin, float outMax);

  // Run one step. error = setpoint - measurement (both in counts).
  // Returns the clamped control output. dt in seconds.
  float compute(float setpoint, float measurement, float dt);

  // Clear integrator and derivative history (call on (re)start of a move
  // or on a state change into FAULT).
  void reset();

  // Live re-tuning during bring-up.
  void setGains(float kp, float ki, float kd);

private:
  float _kp, _ki, _kd;
  float _outMin, _outMax;
  float _integral;   // accumulated I term (anti-windup clamped)
  float _prevError;  // for the D term
};
