#include "GCodeParser.h"

#include <ctype.h>
#include <stdlib.h>

#include "Pins.h"

int GcodeParserFull(GCodeCommand& command, Manager& manager) {
  if (!ReadSerialInput(command)) {
    return kNoEvent;  // no complete line yet this call — try again next loop()
  }

  if (!SendToController(command, manager)) {
    return kNoEvent;  // failed validation; error already printed
  }

  if (command.hasX() || command.hasY() || command.hasF()) {  // check this
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

// Non-blocking: drains whatever bytes are currently available, returns
// immediately either way. Returns true only once a full line has been
// accumulated AND successfully parsed into `command`.
bool ReadSerialInput(GCodeCommand& command) {
  static char buffer[128];
  static size_t index = 0;

  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (index == 0) continue;  // ignore blank lines / stray \r\n
      buffer[index] = '\0';
      index = 0;
      if (Parser(buffer, command)) {
        return true;  // got a complete, meaningful command — hand it back now
      }
      // line parsed but produced nothing meaningful — keep draining
    } else if (index < sizeof(buffer) - 1) {
      buffer[index++] = c;
    }
  }
  return false;  // no complete line finished this call
}

bool Parser(char* in, GCodeCommand& out) {
  out.setType(GCodeCommand::IDLE);  // require an explicit G/M each line
  out.HasX(false);
  out.HasY(false);
  // F deliberately not cleared — inherited from the previous command

  char* p = in;
  bool sawAnyToken = false;

  while (*p != '\0' && *p != ';') {
    if (isspace(static_cast<unsigned char>(*p))) {
      ++p;
      continue;
    }

    char letter = static_cast<char>(toupper(static_cast<unsigned char>(*p)));
    ++p;

    // The very first token on the line must be G or M.
    if (!sawAnyToken && letter != 'G' && letter != 'M') {
      out.setType(GCodeCommand::UNKNOWN);
      return true;  // let SendToController produce the "Unknown command" error
    }
    sawAnyToken = true;

    if (letter == 'X') {
      out.setX(static_cast<float>(strtod(p, &p)));
    } else if (letter == 'Y') {
      out.setY(static_cast<float>(strtod(p, &p)));
    } else if (letter == 'F') {
      out.setF(static_cast<float>(strtod(p, &p)));
    } else if (letter == 'G') {
      out.setCommandTypeFromValue(static_cast<int>(strtod(p, &p)));
    } else if (letter == 'M') {
      out.setCommandTypeFromValue(static_cast<int>(strtod(p, &p) * 10));
    }
  }

  return out.getType() != GCodeCommand::IDLE || out.hasX() || out.hasY() ||
         out.hasF();
}

bool SendToController(GCodeCommand& command, Manager& manager) {
  static bool feedRateEverSet = false;

  if (command.getType() == GCodeCommand::UNKNOWN) {
    Serial.println("Error: Unknown command type. Please try again.");
    command.reset();
    return false;
  }

  if (command.getType() == GCodeCommand::IDLE) {
    Serial.println("Error: No G/M command specified. Please try again.");
    command.reset();
    return false;
  }

  if (command.getType() == GCodeCommand::MOVE_G1 &&
      (!command.hasX() || !command.hasY())) {
    Serial.println("Error: G1 requires both X and Y. Please try again.");
    command.reset();
    return false;
  }

  if (command.getType() == GCodeCommand::MOVE_G1 &&
      !command.hasF() && !feedRateEverSet) {
    Serial.println("Error: F must be specified on the first move command.");
    command.reset();
    return false;
  }

  if (command.hasF() && command.getF() <= 0) {
    Serial.println("Error: F value cannot be zero or negative. Please try again.");
    command.reset();
    return false;
  }

  // Reject a feed rate the machine can't actually reach, in mm/min.
  const float maxFeedMmPerMin =
      (cfg::MAX_VEL_CPS / cfg::COUNTS_PER_MM) * 60.0f;

  if (command.hasF() && command.getF() > maxFeedMmPerMin) {
    Serial.print("Error: F exceeds maximum feed rate of ");
    Serial.print(maxFeedMmPerMin, 1);
    Serial.println(" mm/min. Please try again.");
    command.reset();
    return false;
  }

  // if (!isCommandWithinBounds(command, manager)) {
  //   Serial.println("Error: Command is outside the workspace bounds. Please try again.");
  //   command.reset();
  //   return false;
  // }

  if (command.hasF()) {
    feedRateEverSet = true;
  }

  return true;
}

bool isCommandWithinBounds(const GCodeCommand& command,
                           const Manager& manager) {
  if (command.hasX() &&
      ((command.getX() + manager.getCurrentX() < 0) ||
       (command.getX() + manager.getCurrentX() > cfg::X_MAX_MM))) {
    return false;
  }
  if (command.hasY() &&
      ((command.getY() + manager.getCurrentY() < 0) ||
       (command.getY() + manager.getCurrentY() > cfg::Y_MAX_MM))) {
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
    case GCodeCommand::FAULT:
    case GCodeCommand::UNKNOWN:
      return -1;
    case GCodeCommand::IDLE:
    default:
      return kNoEvent;
  }
}