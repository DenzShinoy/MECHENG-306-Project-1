#include "manager.h"

Manager::Manager() : curr_command{0, 0}, feed_rate(0), curr_event(0) {}

void Manager::setCommand(int x, int y) {
  curr_command.x = x;
  curr_command.y = y;
}

Command Manager::getCommand() const { return curr_command; }

void Manager::setFeedRate(int rate) { feed_rate = rate; }

int Manager::getFeedRate() const { return feed_rate; }

void Manager::setEvent(int event) { curr_event = event; }

int Manager::getEvent() const { return curr_event; }

void Manager::setCurrentPosition(long x, long y) {
  current_x_ = x;
  current_y_ = y;
}

long Manager::getCurrentX() { return current_x_; }

long Manager::getCurrentY() { return current_y_; }