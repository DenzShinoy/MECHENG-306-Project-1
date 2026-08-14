#include "GCodeParser.h"

#include <ctype.h>
#include <stdlib.h>

#include "Pins.h"

int GcodeParserFull(GCodeCommand& command, Manager& manager) {
  ReadSerialInput(command);

  if (!SendToController(command, manager)) {
    return kNoEvent;  // failed validation; error already printed
  }

  if (command.hasX() || command.hasY()) {
    manager.setCommand(static_cast<int>(command.getX()),
                       static_cast<int>(command.getY()));
  }
  if (command.hasF()) {
    manager.setFeedRate(command.getF());
  }

  int event = EventFromCommand(command);
  if (event != kNoEvent) {
    manager.setEvent(event);
  }
  return event;
}

bool ReadSerialInput(GCodeCommand& command) {
  static char buffer[128];
  static size_t index = 0;

  while (true) {  // CANNOT HAVE THIS BLOCKING, CHANGE THIS
    while (Serial.available() == 0) {
      // block until at least one byte arrives
    }

    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (index == 0) continue;  // ignore blank lines / stray \r\n
      buffer[index] = '\0';
      index = 0;
      if (Parser(buffer, command)) {
        return true;
      }
      // parsed but produced nothing meaningful — keep waiting
    } else if (index < sizeof(buffer) - 1) {
      buffer[index++] = c;
    }
  }
}

bool Parser(char* in, GCodeCommand& out) {
  out.HasX(false);
  out.HasY(false);
  // F deliberately not cleared — inherited from the previous command
  // if this line doesn't specify one.

  char* p = in;
  while (*p != '\0' && *p != ';') {
    if (isspace(static_cast<unsigned char>(*p))) {
      ++p;
      continue;
    }

    char letter = static_cast<char>(toupper(static_cast<unsigned char>(*p)));
    ++p;

    if (letter == 'X') {
      out.setX(static_cast<float>(strtod(p, &p)));
    } else if (letter == 'Y') {
      out.setY(static_cast<float>(strtod(p, &p)));
    } else if (letter == 'F') {
      out.setF(static_cast<float>(strtod(p, &p)));
    } else if (letter == 'G') {
      out.setCommandTypeFromValue(static_cast<int>(strtod(p, &p)));
    } else if (letter == 'M') {
      out.setCommandTypeFromValue(static_cast<int>(strtod(p, &p) * 100));
    }
  }

  return out.getType() != GCodeCommand::IDLE || out.hasX() || out.hasY() ||
         out.hasF();
}

bool SendToController(GCodeCommand& command, Manager& manager) {
  if (command.getType() == GCodeCommand::UNKNOWN) {
    Serial.println("Error: Unknown command type. Please try again.");
    command.reset();
    return false;
  }

  if (command.hasX() && command.getX() < 0) {
    Serial.println("Error: X value cannot be negative. Please try again.");
    command.reset();
    return false;
  }

  if (command.hasY() && command.getY() < 0) {
    Serial.println("Error: Y value cannot be negative. Please try again.");
    command.reset();
    return false;
  }

  if (command.hasF() && command.getF() < 0) {
    Serial.println("Error: F value cannot be negative. Please try again.");
    command.reset();
    return false;
  }

  if (!isCommandWithinBounds(command, manager)) {
    Serial.println(
        "Error: Command is outside the workspace bounds. Please try again.");
    command.reset();
    return false;
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