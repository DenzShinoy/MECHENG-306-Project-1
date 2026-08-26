#include "GCodeParser.h"

#include <ctype.h>
#include <stdlib.h>

#include "Pins.h"

// =====================================================================
// Full G-code parser interface
// =====================================================================

int GcodeParserFull(GCodeCommand& command, Manager& manager) {
  if (!ReadSerialInput(command)) {
    // No complete valid line yet.
    return kNoEvent;
  }

  if (!SendToController(command, manager)) {
    // Command failed validation.
    return kNoEvent;
  }

  // Store motion parameters in the manager.
  //
  // F is modal, so command.getF() may contain the feed rate inherited
  // from an earlier G1 command.
  if (command.hasX() || command.hasY() || command.hasF()) {
    manager.setCommand(static_cast<int>(command.getX()),
                       static_cast<int>(command.getY()),
                       static_cast<int>(command.getF()));
  }

  int event = EventFromCommand(command);

  if (event != kNoEvent) {
    manager.setEvent(event);
  }

  return event;
}

// =====================================================================
// Serial input
// =====================================================================
//
// Non-blocking serial reader.
//
// Improvements:
// - Ignores non-printable serial garbage.
// - Ignores stray characters before a G or M.
// - Protects against buffer overflow.
// - Waits for a complete line before parsing.
// - CR and LF are both accepted as line endings.
//
// Example:
//
//   garbage?G1X10Y10F2600
//
// becomes:
//
//   G1X10Y10F2600
//
// =====================================================================

bool ReadSerialInput(GCodeCommand& command) {
  static char buffer[128];
  static size_t index = 0;

  // If the line gets too long, ignore everything until the next newline.
  static bool discardLine = false;

  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());

    // ---------------------------------------------------------------
    // End of line
    // ---------------------------------------------------------------

    if (c == '\n' || c == '\r') {
      // If this line overflowed, throw the entire thing away.
      if (discardLine) {
        index = 0;
        discardLine = false;
        continue;
      }

      // Ignore empty CR/LF characters.
      if (index == 0) {
        continue;
      }

      // Null terminate the command.
      buffer[index] = '\0';
      index = 0;

      // Debug: shows exactly what reached the parser.
      Serial.print(F("RAW: ["));
      Serial.print(buffer);
      Serial.println(F("]"));

      // Parser returns true only if it produced a command that should
      // continue through validation.
      if (Parser(buffer, command)) {
        Serial.print(F("Parsed type: "));
        Serial.println(static_cast<int>(command.getType()));

        return true;
      }

      // Malformed line was ignored.
      // Keep checking serial in case another complete command is waiting.
      continue;
    }

    // ---------------------------------------------------------------
    // Ignore non-printable serial garbage
    // ---------------------------------------------------------------

    if (!isprint(static_cast<unsigned char>(c))) {
      continue;
    }

    // ---------------------------------------------------------------
    // Before a command starts, ignore everything until G or M
    // ---------------------------------------------------------------

    if (index == 0) {
      // Ignore spaces/tabs before the command.
      if (isspace(static_cast<unsigned char>(c))) {
        continue;
      }

      char first = static_cast<char>(toupper(static_cast<unsigned char>(c)));

      // Every supported command must begin with G or M.
      //
      // Therefore random serial noise before the command cannot cause
      // UNKNOWN.
      if (first != 'G' && first != 'M') {
        Serial.print(F("Ignoring stray serial character: "));
        Serial.println(c);
        continue;
      }

      // Store command letters consistently as uppercase.
      c = first;
    }

    // ---------------------------------------------------------------
    // Add character to buffer
    // ---------------------------------------------------------------

    if (index < sizeof(buffer) - 1) {
      buffer[index++] = c;
    } else {
      // Line exceeded the buffer.
      //
      // Do not parse a truncated command. Ignore everything until the
      // next newline and then begin fresh.
      Serial.println(F("Serial line too long. Discarding line."));

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

// Parses a plain integer starting at p.
//
// Valid:
//   10
//   -10
//   +10
//
// Invalid:
//   10.5
//   abc
//   -
//
// Decimals are deliberately rejected for X/Y/F/G/M.
bool parseIntToken(char*& p, float& out) {
  Serial.print(F("parseIntToken input: \""));
  Serial.print(p);
  Serial.println(F("\""));

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
    Serial.println(F("parseIntToken: no digit found, rejecting"));

    p = start;
    return false;
  }

  // Decimals are not allowed.
  //
  // Reject 10.5 instead of silently interpreting it as 10.
  if (*p == '.') {
    Serial.println(F("parseIntToken: decimal point not allowed, rejecting"));

    p = start;
    return false;
  }

  out = negative ? -static_cast<float>(value) : static_cast<float>(value);

  return true;
}

}  // namespace

// =====================================================================
// G-code parser
// =====================================================================

bool Parser(char* in, GCodeCommand& out) {
  // Reset values which belong specifically to this line.
  out.setType(GCodeCommand::IDLE);

  out.HasX(false);
  out.HasY(false);

  // IMPORTANT:
  //
  // Do NOT clear F here.
  //
  // F is modal. Once F has been set it remains active until a full
  // command.reset() occurs, for example when M999 performs a full reset.

  char* p = in;
  bool sawAnyToken = false;

  while (*p != '\0' && *p != ';') {
    // Ignore whitespace between tokens.
    if (isspace(static_cast<unsigned char>(*p))) {
      ++p;
      continue;
    }

    char letter = static_cast<char>(toupper(static_cast<unsigned char>(*p)));

    ++p;

    // ---------------------------------------------------------------
    // First token must be G or M
    // ---------------------------------------------------------------

    if (!sawAnyToken && letter != 'G' && letter != 'M') {
      Serial.println(F("Malformed command: first token is not G/M. Ignoring."));

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
        Serial.println(F("Invalid X value. Command ignored."));

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
        Serial.println(F("Invalid Y value. Command ignored."));

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
        Serial.println(F("Invalid F value. Command ignored."));

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
        Serial.println(F("Invalid G value. Command ignored."));

        out.resetLine();
        return false;
      }

      Serial.print(F("G value parsed as: "));
      Serial.println(val);

      out.setCommandTypeFromValue(static_cast<int>(val));

      Serial.print(F("Type after setCommandTypeFromValue: "));
      Serial.println(static_cast<int>(out.getType()));

      // NOTE:
      //
      // If the number itself is valid but unsupported, for example G55,
      // setCommandTypeFromValue() sets UNKNOWN.
      //
      // We deliberately leave that as UNKNOWN so SendToController()
      // can report a genuine unsupported G-code command.
    }

    // ---------------------------------------------------------------
    // M
    // ---------------------------------------------------------------

    else if (letter == 'M') {
      if (!parseIntToken(p, val)) {
        Serial.println(F("Invalid M value. Command ignored."));

        out.resetLine();
        return false;
      }

      out.setCommandTypeFromValue(static_cast<int>(val * 10));
    }

    // ---------------------------------------------------------------
    // Anything else
    // ---------------------------------------------------------------

    else {
      // This is malformed input rather than a supported/unsupported
      // G-code command.
      //
      // Ignore the whole line rather than generating an UNKNOWN event.
      Serial.print(F("Invalid character in command: "));
      Serial.print(letter);
      Serial.println(F(". Command ignored."));

      out.resetLine();
      return false;
    }
  }

  // Nothing useful was parsed.
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
  // ---------------------------------------------------------------
  // Valid G/M number, but unsupported command
  // ---------------------------------------------------------------

  if (command.getType() == GCodeCommand::UNKNOWN) {
    Serial.println(F("Error: Unknown command type. Please try again."));

    command.resetLine();
    return false;
  }

  // ---------------------------------------------------------------
  // No G/M command
  // ---------------------------------------------------------------

  if (command.getType() == GCodeCommand::IDLE) {
    Serial.println(F("Error: No G/M command specified. Please try again."));

    command.resetLine();
    return false;
  }

  // ---------------------------------------------------------------
  // G1 requires both X and Y
  // ---------------------------------------------------------------

  if (command.getType() == GCodeCommand::MOVE_G1 &&
      (!command.hasX() || !command.hasY())) {
    Serial.println(F("Error: G1 requires both X and Y. Please try again."));

    command.resetLine();
    return false;
  }

  // ---------------------------------------------------------------
  // Feed rate
  // ---------------------------------------------------------------
  //
  // hasF() remains true once a valid F has been supplied.
  //
  // Therefore:
  //
  //   G1X10Y10F2600
  //   G1X20Y20
  //
  // causes the second command to inherit F2600.
  //
  // A full command.reset(), such as M999, clears hasF().

  if (command.getType() == GCodeCommand::MOVE_G1 && !command.hasF()) {
    Serial.println(F("Error: F must be specified on the first move command."));

    command.resetLine();
    return false;
  }

  if (command.hasF() && command.getF() <= 0) {
    Serial.println(
        F("Error: F value cannot be zero or negative. Please try again."));

    command.resetLine();
    return false;
  }

  // ---------------------------------------------------------------
  // Maximum feed rate
  // ---------------------------------------------------------------

  const float maxFeedMmPerMin = (cfg::MAX_VEL_CPS / cfg::COUNTS_PER_MM) * 60.0f;

  if (command.hasF() && command.getF() > maxFeedMmPerMin) {
    Serial.print(F("Warning: F exceeds maximum feed rate. Clamping to "));

    Serial.print(maxFeedMmPerMin, 1);
    Serial.println(F(" mm/min."));

    command.setF(maxFeedMmPerMin);
  }

  // ---------------------------------------------------------------
  // Workspace bounds
  // ---------------------------------------------------------------

  // if (command.getType() == GCodeCommand::MOVE_G1 &&
  //     !isCommandWithinBounds(command, manager)) {
  //   const long proposedX =
  //       manager.getCurrentX() + static_cast<long>(command.getX());

  //   const long proposedY =
  //       manager.getCurrentY() + static_cast<long>(command.getY());

  //   Serial.print(F("Error: Move outside workspace. Current X="));
  //   Serial.print(manager.getCurrentX());
  //   Serial.print(F(" Y="));
  //   Serial.print(manager.getCurrentY());

  //   Serial.print(F(" Requested X="));
  //   Serial.print(proposedX);
  //   Serial.print(F(" Y="));
  //   Serial.println(proposedY);

  //   command.resetLine();
  //   return false;
  // }

  return true;
}

// =====================================================================
// Workspace bounds
// =====================================================================

bool isCommandWithinBounds(const GCodeCommand& command,
                           const Manager& manager) {
  const long proposedX =
      manager.getCurrentX() + static_cast<long>(command.getX());

  const long proposedY =
      manager.getCurrentY() + static_cast<long>(command.getY());

  if (proposedX < 0 || proposedX > cfg::X_MAX_MM) {
    return false;
  }

  if (proposedY < 0 || proposedY > cfg::Y_MAX_MM) {
    return false;
  }

  return true;
}

// =====================================================================
// Convert G-code command to FSM event
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