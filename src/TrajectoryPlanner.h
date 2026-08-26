#pragma once
#include <Arduino.h>
#include "Kinematics.h"

// =====================================================================
//  Module 6 — TrajectoryPlanner
// ---------------------------------------------------------------------
//  Trapezoidal velocity profile over a straight Cartesian move, our own
//  code. Coordinated 2-axis: both axes share one time base so they start
//  and finish together (the dominant axis sets the profile; the other is
//  scaled to it), giving a straight line in XY. plan() sets up a move;
//  update() is polled with the current time and yields the interpolated
//  setpoint (in COUNTS) that the per-axis PIDs chase. Non-blocking.
// =====================================================================

class TrajectoryPlanner {
public:
  // maxVel in counts/s, maxAcc in counts/s^2 (see cfg:: defaults).
  TrajectoryPlanner(float maxVel, float maxAcc);

  // Set up a move from the current Cartesian point to a target (counts).
  // Computes accel / cruise / decel phases and total duration.
  void plan(const Point& start, const Point& target, uint32_t nowMs);

  // Advance the profile to nowMs and update the interpolated setpoint.
  // Call every loop.
  void update(uint32_t nowMs);

  // Current commanded setpoint along the path (Cartesian counts).
  Point setpoint() const;

  // True once the profile has reached the target.
  bool isDone() const;

private:
  const float _maxVel;
  const float _maxAcc;

  Point    _start;
  Point    _target;
  Point    _setpoint;   // current interpolated command
  uint32_t _startMs;    // when the move began
  float    _totalTime;  // profile duration, seconds
  bool     _done;
};
