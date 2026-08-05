#pragma once

#include "GCodeParser.h"
#include "Kinematics.h"

class PositionTracker {
 public:
  PositionTracker(float maxX, float maxY) : maxX_(maxX), maxY_(maxY) {}

  void reset();
  float getX() const;
  float getY() const;
  void updateFromEncoders(long leftCounts, long rightCounts);
  bool isCommandWithinBounds(const GCodeCommand& command) const;

 private:
  float x_ = 0.0f;
  float y_ = 0.0f;
  float maxX_;
  float maxY_;
};
