#include "manager.h"

<<<<<<< HEAD
Manager::Manager()
    : curr_command{0, 0}, curr_event(0), current_x_(0), current_y_(0) {}
=======
#include "GCodeParser.h"
>>>>>>> 3d2ac2e (updated some stuff)

GCodeCommand gcode_command;  // Global instance of GCodeCommand

GCodeCommand getGCodeCommand() { return gcode_command; }

Manager::Manager() : curr_command{0, 0}, curr_event(0) {}

Command Manager::getCommand() const { return curr_command; }

<<<<<<< HEAD
=======
int Manager::getFeedRate() const { return feed_rate; }

>>>>>>> 3d2ac2e (updated some stuff)
void Manager::setEvent(int event) { curr_event = event; }

int Manager::getEvent() const { return curr_event; }

<<<<<<< HEAD
void Manager::setCurrentPosition(long x, long y) {
  current_x_ += x;
  current_y_ += y;
}

long Manager::getCurrentX() const { return current_x_; }
long Manager::getCurrentY() const { return current_y_; }
=======
long Manager::getCurrentX() { return current_x_; }

long Manager::getCurrentY() { return current_y_; }
>>>>>>> 3d2ac2e (updated some stuff)
