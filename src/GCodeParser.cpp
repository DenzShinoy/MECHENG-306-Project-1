#include "GCodeParser.h"

#include <ctype.h>
#include <stdlib.h>

GCodeCommand command;

void ReadSerialInput() {
  static char buffer[128];
  static size_t index = 0;

  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      buffer[index] = '\0';  // Null-terminate the string
      if (Parser(buffer, command)) {
        // Process the command
      }
      index = 0;  // Reset index for the next line
    } else {
      if (index < sizeof(buffer) - 1) {
        buffer[index++] = c;  // Store character in buffer
      }
    }
  }
}

bool Parser(char* in, GCodeCommand& out) {
  out.HasX(false);
  out.HasY(false);
  // no F as F value is inherited from previous command if not specified in the
  // current command

  char* p = in;
  while (*p != '\0' && *p != ';') {
    if (isspace(static_cast<unsigned char>(*p))) {  // Skip whitespace
      ++p;
      continue;
    }

    char letter = static_cast<char>(toupper(static_cast<unsigned char>(
        *p)));  // get letter and convert to uppercase
    ++p;

    if (letter == 'X') {
      out.setX(static_cast<float>(strtod(p, &p)));
      out.HasX(true);
    } else if (letter == 'Y') {
      out.setY(static_cast<float>(strtod(p, &p)));
      out.HasY(true);
    } else if (letter == 'F') {
      out.setF(static_cast<float>(strtod(p, &p)));
      out.HasF(true);
    } else if (letter == 'G') {
      out.setCommandTypeFromValue(static_cast<int>(strtod(p, &p)));
    } else if (letter == 'M') {
      out.setCommandTypeFromValue(static_cast<int>(strtod(p, &p) * 100));
    }
  }

  return out.getType() != GCodeCommand::IDLE || out.hasX() || out.hasY() ||
         out.hasF();
}

bool SendToController() {

  // Do error checking first, if any are found, reset all values and prompt to try again
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

  // do a position check to ensure the command is within the bounds of the plotter's workspace
  // This is a placeholder for the actual position checking logic
  // if (!isWithinBounds(command)) {
  //   Serial.println("Error: Command is outside the workspace bounds. Please try again.");
  //   command.reset();
  //   return false;
  // }

  // If all checks pass, pass into move state to allow controller to read command values and execute the move


  return true;
}