#pragma once
#include <Arduino.h>

// =====================================================================
//  Module 8 — GCodeParser
// ---------------------------------------------------------------------
//  Parse one line of G-code into a command struct, our own code. This is
//  the mm boundary: X/Y/F fields are millimetres and mm/min as written
//  by the host; downstream modules convert to counts via Kinematics.
//  Fixed-size, no dynamic allocation, no String. Supports G1 (move) and
//  G28 (home); anything else parses as UNKNOWN.
// =====================================================================

struct GCodeCommand {
  enum Type : uint8_t { NONE, MOVE_G1, HOME_G28, UNKNOWN };

  Type  type = NONE;
  float x = 0.0f;   // mm, valid only if hasX
  float y = 0.0f;   // mm, valid only if hasY
  float f = 0.0f;   // feedrate mm/min, valid only if hasF
  bool  hasX = false;
  bool  hasY = false;
  bool  hasF = false;
};

class GCodeParser {
public:
  GCodeParser() = default;

  // Parse one NUL-terminated line into `out`. Returns true if a command
  // was recognised (G1/G28); false for blank/comment/garbage lines.
  // Does not mutate any state — pure function over the input line.
  bool parseLine(const char* line, GCodeCommand& out) const;
};
