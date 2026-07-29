#include "GCodeParser.h"

bool GCodeParser::parseLine(const char* line, GCodeCommand& out) const {
  // TODO: skip blanks/comments (';'); read the leading G-word; for G1
  // scan X/Y/F words with strtod and set the has* flags; map G28 to
  // HOME_G28. Return false when nothing recognisable is found.
  (void)line; (void)out;
  return false;
}
