#pragma once
#include <Arduino.h>

#include "manager.h"

struct GCodeCommand {
 public:
  enum Type : uint8_t { IDLE, MOVE_G1, HOME_G28, FAULT, UNKNOWN, CLEAR_FAULT };

  GCodeCommand() = default;

  Type getType() const { return type_; }
  float getX() const { return x_; }
  float getY() const { return y_; }
  float getF() const { return f_; }
  bool hasX() const { return hasX_; }
  bool hasY() const { return hasY_; }
  bool hasF() const { return hasF_; }

  // The G/M code exactly as it was typed, kept so a rejected line can
  // name itself. setCommandTypeFromValue() folds M999 into 9990 and
  // throws everything unrecognised into one UNKNOWN bucket, so the
  // original number is gone by the time anyone wants to report it.
  char getCodeLetter() const { return codeLetter_; }
  int getCodeValue() const { return codeValue_; }
  void setCode(char letter, int value) {
    codeLetter_ = letter;
    codeValue_ = value;
  }

  void resetLine() {
    type_ = IDLE;
    x_ = 0.0f;
    y_ = 0.0f;

    hasX_ = false;
    hasY_ = false;

    // Keep f_ and hasF_
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
    } else if (in == (999 * 10)) {
      setType(CLEAR_FAULT);  // was: reset()
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
  char codeLetter_ = '?';
  int codeValue_ = 0;
};

// Sentinel: this call didn't produce a state-changing event.
constexpr int kNoEvent = -2;

bool ReadSerialInput(GCodeCommand& command);
bool Parser(char* in, GCodeCommand& out);
bool SendToController(GCodeCommand& command, Manager& manager);
bool isCommandWithinBounds(const GCodeCommand& command, const Manager& manager);
int EventFromCommand(const GCodeCommand& command);

// Blocks until a valid line is read, validates it, pushes X/Y/F into
// `manager`, and returns the FSM event it implies (or kNoEvent if the
// line failed validation and nothing changed).
int GcodeParserFull(GCodeCommand& command, Manager& manager);