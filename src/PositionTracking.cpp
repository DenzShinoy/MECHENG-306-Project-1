#include "PositionTracking.h"

#include "Pins.h"

void PositionTracker::reset() {
  x_ = 0.0f;
  y_ = 0.0f;
}

float PositionTracker::getX() const {
  return x_;
}

float PositionTracker::getY() const {
  return y_;
}

//
// just read encoder values from PID Loop and update the position tracker, probably can alter this function
void PositionTracker::updateFromEncoders(long leftCounts, long rightCounts) {
  const AxisPair motorCounts{leftCounts, rightCounts};
  const Point cartesianCounts = Kinematics::abToXY(motorCounts);

  x_ = Kinematics::countsToMm(cartesianCounts.x);
  y_ = Kinematics::countsToMm(cartesianCounts.y);
}

bool PositionTracker::isCommandWithinBounds(const GCodeCommand& command) const {
  const float targetX = command.hasX() ? (x_ + command.getX()) : x_;
  const float targetY = command.hasY() ? (y_ + command.getY()) : y_;

  return targetX >= 0.0f && targetX <= maxX_ && targetY >= 0.0f &&
         targetY <= maxY_;
}
