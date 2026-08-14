#include "manager.h"

Manager::Manager()
    : curr_command{0, 0}, curr_event(0), current_x_(0), current_y_(0) {}

void Manager::setCommand(int x, int y) {
  curr_command.x = x;
  curr_command.y = y;
}

Command Manager::getCommand() const { return curr_command; }

void Manager::setFeedRate(int rate) { (void)rate; }

int Manager::getFeedRate() const { return 0; }

void Manager::setEvent(int event) { curr_event = event; }

int Manager::getEvent() const { return curr_event; }

void Manager::setCurrentPosition(long x, long y) {
  current_x_ = x;
  current_y_ = y;
}

long Manager::getCurrentX() { return current_x_; }

long Manager::getCurrentY() { return current_y_; }