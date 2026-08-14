#include "manager.h"

Manager::Manager()
    : curr_command{0, 0}, curr_event(0), current_x_(0), current_y_(0) {}

void Manager::setCommand(int x, int y) {
  curr_command.x = x;
  curr_command.y = y;
}

Command Manager::getCommand() const { return curr_command; }

void Manager::setFeedRate(int rate) { feed_rate_ = rate; }
int Manager::getFeedRate() const { return feed_rate_; }

void Manager::setEvent(int event) { curr_event = event; }
int Manager::getEvent() const { return curr_event; }

void Manager::setCurrentPosition(long x, long y) {
  current_x_ += x;
  current_y_ += y;
}

long Manager::getCurrentX() const { return current_x_; }
long Manager::getCurrentY() const { return current_y_; }