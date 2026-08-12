#pragma once
#include <Arduino.h>

// =====================================================================
//  Module 8 — GCodeParser
// ---------------------------------------------------------------------
//  Parse one line of G-code into a command struct, our own code. This is
//  the mm boundary: X/Y fields are millimetres and mm/min as written
//  by the host; downstream modules convert to counts via Kinematics.
//  Fixed-size, no dynamic allocation, no String. Supports G1 (move) and
//  G28 (home); anything else parses as UNKNOWN.
// =====================================================================


struct GCodeCommand;
bool Parser(char* in, GCodeCommand& out);

struct GCodeCommand {
 public:
  enum Type : uint8_t { IDLE, MOVE_G1, HOME_G28, FAULT, UNKNOWN };
  // change a pin / trigger an ISR to then change the state of the FSM

  GCodeCommand() = default;
//
  Type getType() const { return type_; }
  float getX() const { return x_; }
  float getY() const { return y_; }
  float getF() const { return f_; }
  bool hasX() const { return hasX_; }
  bool hasY() const { return hasY_; }
  bool hasF() const { return hasF_; }

  void reset() {
    type_ = IDLE;
    x_ = 0.0f;
    y_ = 0.0f;
    f_ = 0.0f;
    hasX_ = false;
    hasY_ = false;
    hasF_ = false;
  }

  void setType(Type type) { type_ = type; }
  void setX(float value) {
    x_ = value;
    hasX_ = true;
  }
  void setY(float value) {
    y_ = value;
    hasY_ = true;
  }
  void setF(float value) {
    f_ = value;
    hasF_ = true;
  }
  void HasX(bool value) { hasX_ = value; }
  void HasY(bool value) { hasY_ = value; }
  void HasF(bool value) { hasF_ = value; }

  void setCommandTypeFromValue(int in) {
    if (in == 1) {
      setType(MOVE_G1);
    } else if (in == 28) {
      setType(HOME_G28);
    } else if (in == 333) {
      setType(FAULT);
    } else if (in == (999 * 100)) {
      reset();
    } else {
      setType(UNKNOWN);
    }
  }

 private:
  Type type_ = IDLE;
  float x_ = 0.0f;
  float y_ = 0.0f;
  float f_ = 0.0f;
  bool hasX_ = false;
  bool hasY_ = false;
  bool hasF_ = false;
};
