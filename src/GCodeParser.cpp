#include "GCodeParser.h"

#include <ctype.h>
#include <stdlib.h>

#include "Pins.h"

// The whole parser front to back, called once per loop.

int GcodeParserFull(GCodeCommand& command, Manager& manager) {
  if (!ReadSerialInput(command)) {
    return kNoEvent;
  }

  if (!SendToController(command, manager)) {
    return kNoEvent;
  }

  if (command.hasX() || command.hasY() || command.hasF()) {
    manager.setCommand(static_cast<int>(command.getX()),
                       static_cast<int>(command.getY()),
                       static_cast<int>(command.getF()));
  }

  return EventFromCommand(command);
}

bool ReadSerialInput(GCodeCommand& command) {
  static char buffer[128];
  static size_t index = 0;
  static bool discardLine = false;

  // Set when we bin a character before the line has even started (see the
  // G/M filter further down). Without it, a line of pure junk ends up with
  // an empty buffer and looks exactly like someone hitting Enter, so you'd
  // get silence back for something you definitely typed.
  static bool sawJunk = false;

  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());

    // End of line.
    if (c == '\n' || c == '\r') {
      if (discardLine) {
        Serial.println(F("ERR: unknown command (line too long)"));
        index = 0;
        discardLine = false;
        sawJunk = false;
        continue;
      }

      if (index == 0) {
        // Nothing usable on the line. A bare Enter is fine; a line that
        // had characters but nothing we could start a command with isn't.
        // "\r\n" only complains once, because the pass that ran on the
        // '\r' already cleared the flag.
        if (sawJunk) {
          Serial.println(F("ERR: unknown command"));
          sawJunk = false;
        }

        continue;
      }

      buffer[index] = '\0';
      index = 0;
      sawJunk = false;

      if (Parser(buffer, command)) {
        return true;
      }

      // Started with G or M, but the rest was rubbish: a bad number, a
      // decimal point, or a letter we don't handle. Echo it back so the
      // typo is there to see.
      Serial.print(F("ERR: unknown command: "));
      Serial.println(buffer);

      continue;
    }

    // Backspace / delete. Terminals send one or the other, 8 or 127.
    //
    // Typing "X", then backspace, then "X-35" should leave "X-35" in the
    // buffer, not "X X-35".
    if (c == '\b' || static_cast<unsigned char>(c) == 127) {
      if (index > 0) {
        --index;
      }

      continue;
    }

    // Anything else non-printable is line noise.
    if (!isprint(static_cast<unsigned char>(c))) {
      continue;
    }

    // Until a command has started, throw away everything but G or M.
    if (index == 0) {
      if (isspace(static_cast<unsigned char>(c))) {
        continue;
      }

      char first = static_cast<char>(toupper(static_cast<unsigned char>(c)));

      if (first != 'G' && first != 'M') {
        sawJunk = true;
        continue;
      }

      c = first;
    }

    if (index < sizeof(buffer) - 1) {
      buffer[index++] = c;
    } else {
      index = 0;
      discardLine = true;
    }
  }

  return false;
}

namespace {
bool parseIntToken(char*& p, float& out) {
  // "X-35", "X -35" and "X    -35" should all mean the same thing.
  while (isspace(static_cast<unsigned char>(*p))) {
    ++p;
  }

  char* start = p;
  bool negative = false;

  if (*p == '+' || *p == '-') {
    negative = (*p == '-');
    ++p;
  }

  long value = 0;
  bool sawDigit = false;

  while (isdigit(static_cast<unsigned char>(*p))) {
    value = value * 10 + (*p - '0');
    ++p;
    sawDigit = true;
  }

  if (!sawDigit) {
    p = start;
    return false;
  }

  if (*p == '.') {
    p = start;
    return false;
  }

  out = negative ? -static_cast<float>(value) : static_cast<float>(value);

  return true;
}
}  // namespace

bool Parser(char* in, GCodeCommand& out) {
  out.setType(GCodeCommand::IDLE);

  out.HasX(false);
  out.HasY(false);

  // F is modal, so don't clear hasF_ here.

  char* p = in;
  bool sawAnyToken = false;

  while (*p != '\0' && *p != ';') {
    if (isspace(static_cast<unsigned char>(*p))) {
      ++p;
      continue;
    }

    char letter = static_cast<char>(toupper(static_cast<unsigned char>(*p)));

    ++p;

    // First token must be G or M.
    if (!sawAnyToken && letter != 'G' && letter != 'M') {
      out.resetLine();
      return false;
    }

    sawAnyToken = true;

    float val;

    if (letter == 'X') {
      if (!parseIntToken(p, val)) {
        out.resetLine();
        return false;
      }

      out.setX(val);
    }

    else if (letter == 'Y') {
      if (!parseIntToken(p, val)) {
        out.resetLine();
        return false;
      }

      out.setY(val);
    }

    else if (letter == 'F') {
      if (!parseIntToken(p, val)) {
        out.resetLine();
        return false;
      }

      out.setF(val);
    }

    else if (letter == 'G') {
      if (!parseIntToken(p, val)) {
        out.resetLine();
        return false;
      }

      out.setCode('G', static_cast<int>(val));
      out.setCommandTypeFromValue(static_cast<int>(val));
    }

    else if (letter == 'M') {
      if (!parseIntToken(p, val)) {
        out.resetLine();
        return false;
      }

      out.setCode('M', static_cast<int>(val));
      out.setCommandTypeFromValue(static_cast<int>(val * 10));
    }

    // Some letter we don't know about.
    else {
      out.resetLine();
      return false;
    }
  }

  if (!sawAnyToken) {
    out.resetLine();
    return false;
  }

  return true;
}

// Everything that can get a line thrown out before it reaches the FSM.

bool SendToController(GCodeCommand& command, Manager& manager) {
  if (command.getType() == GCodeCommand::UNKNOWN) {
    // Parsed fine, but it's a code this machine doesn't do. Print what
    // was typed alongside what we do know, so it's one line to read
    // instead of two.
    Serial.print(F("ERR: unknown command "));
    Serial.print(command.getCodeLetter());
    Serial.print(command.getCodeValue());
    Serial.println(F(", known: G1 G28 G333 M999"));

    command.resetLine();
    return false;
  }

  if (command.getType() == GCodeCommand::IDLE) {
    command.resetLine();
    return false;
  }

  if (command.getType() == GCodeCommand::MOVE_G1 &&
      (!command.hasX() || !command.hasY())) {
    Serial.println(F("ERR: G1 needs both X and Y"));
    command.resetLine();
    return false;
  }

  // F is modal: once it's been given, hasF() stays true for every line
  // after it, because resetLine() keeps f_ and hasF_.
  if (command.getType() == GCodeCommand::MOVE_G1 && !command.hasF()) {
    // Modal, but you can't send it on its own, because the first token of
    // a line has to be a G or an M. So it rides along on a G1, which is
    // what the generated G-code does: F on the first move, never again.
    Serial.println(F("ERR: G1 needs a feed rate: G1 X10 Y10 F1000"));
    command.resetLine();
    return false;
  }

  if (command.hasF() && command.getF() <= 0) {
    Serial.println(F("ERR: F must be above 0"));
    command.resetLine();
    return false;
  }

  // Too fast just gets clamped, rather than the whole line thrown out.
  // G1 may well slow it down further than this for straightness.
  if (command.hasF() && command.getF() > cfg::MAX_FEED_MM_PER_MIN) {
    command.setF(cfg::MAX_FEED_MM_PER_MIN);
  }

  if (command.getType() == GCodeCommand::MOVE_G1 &&
      !isCommandWithinBounds(command, manager)) {
    command.resetLine();
    return false;
  }

  return true;
}

bool isCommandWithinBounds(const GCodeCommand& command,
                           const Manager& manager) {
  const long currentX = manager.getCurrentX();
  const long currentY = manager.getCurrentY();

  const long moveX = static_cast<long>(command.getX());
  const long moveY = static_cast<long>(command.getY());

  const long proposedX = currentX + moveX;
  const long proposedY = currentY + moveY;

  const bool inside = proposedX >= 0 && proposedX <= manager.getMaxX() &&
                      proposedY >= 0 && proposedY <= manager.getMaxY();

  if (!inside) {
    // X/Y are relative to where the pen already is, so print all three
    // numbers: where we are, what was asked for, and where that lands.
    // Otherwise the rejection looks like it came out of nowhere.
    Serial.print(F("ERR: out of bounds: at X:"));
    Serial.print(currentX);
    Serial.print(F(" Y:"));
    Serial.print(currentY);
    Serial.print(F(" + X:"));
    Serial.print(moveX);
    Serial.print(F(" Y:"));
    Serial.print(moveY);
    Serial.print(F(" = X:"));
    Serial.print(proposedX);
    Serial.print(F(" Y:"));
    Serial.print(proposedY);
    Serial.print(F(", limits X:0-"));
    Serial.print(manager.getMaxX());
    Serial.print(F(" Y:0-"));
    Serial.println(manager.getMaxY());

    return false;
  }

  return true;
}

int EventFromCommand(const GCodeCommand& command) {
  switch (command.getType()) {
    case GCodeCommand::MOVE_G1:
      return 1;

    case GCodeCommand::HOME_G28:
      return 2;

    case GCodeCommand::CLEAR_FAULT:
      return 0;

    case GCodeCommand::FAULT:
    case GCodeCommand::UNKNOWN:
      return -1;

    case GCodeCommand::IDLE:
    default:
      return kNoEvent;
  }
}