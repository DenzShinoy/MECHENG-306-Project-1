#include "TrajectoryPlanner.h"

TrajectoryPlanner::TrajectoryPlanner(float maxVel, float maxAcc)
    : _maxVel(maxVel), _maxAcc(maxAcc),
      _start{0, 0}, _target{0, 0}, _setpoint{0, 0},
      _startMs(0), _totalTime(0.0f), _done(true) {}

void TrajectoryPlanner::plan(const Point& start, const Point& target,
                             uint32_t nowMs) {
  // TODO: store endpoints; compute path length; derive trapezoidal
  // accel/cruise/decel timing from _maxVel/_maxAcc (fall back to a
  // triangular profile for short moves); set _totalTime, _startMs, _done.
  (void)start; (void)target; (void)nowMs;
}

void TrajectoryPlanner::update(uint32_t nowMs) {
  // TODO: t = (nowMs - _startMs)/1000; evaluate the profile's distance
  // fraction at t; interpolate _setpoint between _start and _target;
  // set _done when t >= _totalTime.
  (void)nowMs;
}

Point TrajectoryPlanner::setpoint() const {
  return _setpoint;
}

bool TrajectoryPlanner::isDone() const {
  return _done;
}
