#include "GCodeParser.h"

#include <ctype.h>
#include <stdlib.h>

#include "Pins.h"

// =====================================================================
// Full G-code parser interface
// =====================================================================

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

// =====================================================================
// Serial input
// =====================================================================

bool ReadSerialInput(GCodeCommand& command) {
  static char buffer[128];
  static size_t index = 0;
  static bool discardLine = false;

  // Set when a character is thrown away before the line ever starts (see
  // the G/M filter below). Without it, a line of pure junk would end with
  // an empty buffer and be indistinguishable from a bare Enter, and the
  // operator would get silence for something they clearly typed.
  static bool sawJunk = false;

  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());

    // ---------------------------------------------------------------
    // End of line
    // ---------------------------------------------------------------
    if (c == '\n' || c == '\r') {
      if (discardLine) {
        Serial.println(F("ERR: unknown command (line too long)"));
        index = 0;
        discardLine = false;
        sawJunk = false;
        continue;
      }

      if (index == 0) {
        // Nothing usable on the line. A bare Enter is not an error; a
        // line that had characters but none we could start a command
        // with is. Sending "\r\n" only reports once, because the line
        // that ran on '\r' cleared the flag.
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

      // It started with G or M but the tokens made no sense: a bad
      // number, a decimal point, or a letter this parser does not know.
      // Echo the line back so the typo is visible.
      Serial.print(F("ERR: unknown command: "));
      Serial.println(buffer);

      continue;
    }

    // ---------------------------------------------------------------
    // Handle Backspace / Delete
    // ---------------------------------------------------------------
    //
    // Some terminals send:
    //   Backspace = ASCII 8
    //   Delete    = ASCII 127
    //
    // If the user types:
    //
    //   X
    //   <backspace>
    //   X-35
    //
    // this removes the old X from our buffer instead of leaving:
    //
    //   X X-35
    //
    // ---------------------------------------------------------------
    if (c == '\b' || static_cast<unsigned char>(c) == 127) {
      if (index > 0) {
        --index;
      }

      continue;
    }

    // ---------------------------------------------------------------
    // Ignore other non-printable serial garbage
    // ---------------------------------------------------------------
    if (!isprint(static_cast<unsigned char>(c))) {
      continue;
    }

    // ---------------------------------------------------------------
    // Before a command starts, ignore everything until G or M
    // ---------------------------------------------------------------
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

    // ---------------------------------------------------------------
    // Add character to buffer
    // ---------------------------------------------------------------
    if (index < sizeof(buffer) - 1) {
      buffer[index++] = c;
    } else {
      index = 0;
      discardLine = true;
    }
  }

  return false;
}

// =====================================================================
// Integer token parser
// =====================================================================

namespace {
bool parseIntToken(char*& p, float& out) {
  // Allow whitespace between a letter and its number.
  //
  // Examples:
  // X-35
  // X -35
  // X    -35
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

// =====================================================================
// Parser
// =====================================================================

bool Parser(char* in, GCodeCommand& out) {
  out.setType(GCodeCommand::IDLE);

  out.HasX(false);
  out.HasY(false);

  // F deliberately stays modal.
  // Do not clear hasF_ here.

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

    // ---------------------------------------------------------------
    // X
    // ---------------------------------------------------------------
    if (letter == 'X') {
      if (!parseIntToken(p, val)) {
        out.resetLine();
        return false;
      }

      out.setX(val);
    }

    // ---------------------------------------------------------------
    // Y
    // ---------------------------------------------------------------
    else if (letter == 'Y') {
      if (!parseIntToken(p, val)) {
        out.resetLine();
        return false;
      }

      out.setY(val);
    }

    // ---------------------------------------------------------------
    // F
    // ---------------------------------------------------------------
    else if (letter == 'F') {
      if (!parseIntToken(p, val)) {
        out.resetLine();
        return false;
      }

      out.setF(val);
    }

    // ---------------------------------------------------------------
    // G
    // ---------------------------------------------------------------
    else if (letter == 'G') {
      if (!parseIntToken(p, val)) {
        out.resetLine();
        return false;
      }

      out.setCode('G', static_cast<int>(val));
      out.setCommandTypeFromValue(static_cast<int>(val));
    }

    // ---------------------------------------------------------------
    // M
    // ---------------------------------------------------------------
    else if (letter == 'M') {
      if (!parseIntToken(p, val)) {
        out.resetLine();
        return false;
      }

      out.setCode('M', static_cast<int>(val));
      out.setCommandTypeFromValue(static_cast<int>(val * 10));
    }

    // ---------------------------------------------------------------
    // Invalid character
    // ---------------------------------------------------------------
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

// =====================================================================
// Command validation
// =====================================================================

bool SendToController(GCodeCommand& command, Manager& manager) {
  if (command.getType() == GCodeCommand::UNKNOWN) {
    // The line parsed cleanly but named a code this machine does not
    // implement. Report it with the supported set, so the operator can
    // see what was typed and what was expected in one line.
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

  // F is modal.
  //
  // Once F has been specified, command.hasF() stays true for every later
  // line: resetLine() deliberately preserves f_ and hasF_.
  if (command.getType() == GCodeCommand::MOVE_G1 && !command.hasF()) {
    // F is modal but cannot be sent on its own: the first token of a line
    // must be G or M, so F has to ride on a G1 (as the generated G-code
    // does -- it sets F once, on the first move).
    Serial.println(F("ERR: G1 needs a feed rate: G1 X10 Y10 F1200"));
    command.resetLine();
    return false;
  }

  if (command.hasF() && command.getF() <= 0) {
    Serial.println(F("ERR: F must be above 0"));
    command.resetLine();
    return false;
  }

  // ---------------------------------------------------------------
  // Maximum feed rate
  // ---------------------------------------------------------------
  // Throttle a feed rate that exceeds the machine ceiling rather than
  // rejecting the command outright. G1 may slow a move further for
  // straightness (see the guard in G1::execute).
  if (command.hasF() && command.getF() > cfg::MAX_FEED_MM_PER_MIN) {
    command.setF(cfg::MAX_FEED_MM_PER_MIN);
  }

  // ---------------------------------------------------------------
  // Workspace bounds
  // ---------------------------------------------------------------
  if (command.getType() == GCodeCommand::MOVE_G1 &&
      !isCommandWithinBounds(command, manager)) {
    command.resetLine();
    return false;
  }

  return true;
}

// =====================================================================
// Workspace bounds
// =====================================================================

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
    // X/Y are RELATIVE to where the tool already is, so the message has
    // to show all three numbers — where we are, what was asked for, and
    // where that lands — or the envelope check looks arbitrary.
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

// =====================================================================
// Convert command to FSM event
// =====================================================================

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