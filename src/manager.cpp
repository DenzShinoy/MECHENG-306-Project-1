#include "manager.h"

#include "GCodeParser.h"

GCodeCommand gcode_command;  // Global instance of GCodeCommand

GCodeCommand getGCodeCommand() { return gcode_command; }

Manager::Manager() : curr_command{0, 0}, curr_event(0) {}

Command Manager::getCommand() const { return curr_command; }

int Manager::getFeedRate() const { return feed_rate; }

void Manager::setEvent(int event) { curr_event = event; }

int Manager::getEvent() const { return curr_event; }

long Manager::getCurrentX() { return current_x_; }

long Manager::getCurrentY() { return current_y_; }