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

  // The G/M code as typed. Worth keeping, because
  // setCommandTypeFromValue() turns M999 into 9990 and tips everything it
  // doesn't recognise into one UNKNOWN bucket, so by the time we want to
  // complain about a line the original number is long gone.
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

    // f_ and hasF_ deliberately survive this. F is modal.
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

// Returned when a call produced nothing the FSM cares about.
constexpr int kNoEvent = -2;

bool ReadSerialInput(GCodeCommand& command);
bool Parser(char* in, GCodeCommand& out);
bool SendToController(GCodeCommand& command, Manager& manager);
bool isCommandWithinBounds(const GCodeCommand& command, const Manager& manager);
int EventFromCommand(const GCodeCommand& command);

// Reads whatever serial has waiting. If that finishes a valid line, it
// pushes X/Y/F into `manager` and returns the FSM event the line implies.
// Returns kNoEvent if there's no complete line yet, or if the line was
// rejected and nothing changed.
int GcodeParserFull(GCodeCommand& command, Manager& manager);