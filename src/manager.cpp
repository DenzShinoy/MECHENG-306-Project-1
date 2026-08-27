#include "manager.h"

Manager::Manager(LimitSwitch& top, LimitSwitch& bottom, LimitSwitch& left,
                 LimitSwitch& right)
    : curr_command{0, 0, 0},
      top_(top),
      bottom_(bottom),
      left_(left),
      right_(right) {}

void Manager::setCommand(int x, int y, int feed_rate) {
  curr_command.x = x;
  curr_command.y = y;
  curr_command.feed_rate = feed_rate;
}

Command Manager::getCommand() const { return curr_command; }

// Position Management
void Manager::setCurrentPosition(long x, long y) {
  current_x_ += x;
  current_y_ += y;
}

void Manager::resetXY() {
  current_x_ = 0;
  current_y_ = 0;
}

long Manager::getCurrentX() const { return current_x_; }

long Manager::getCurrentY() const { return current_y_; }
int Manager::getMaxX() const { return max_x; }

int Manager::getMaxY() const { return max_y; }

// Limit switch management

void Manager::beginLimits() {
  top_.begin();
  bottom_.begin();
  left_.begin();
  right_.begin();
}

void Manager::updateLimits(uint32_t nowMs) {
  top_.update(nowMs);
  bottom_.update(nowMs);
  left_.update(nowMs);
  right_.update(nowMs);
}

bool Manager::bottomPressed() const { return bottom_.isPressed(); }

bool Manager::leftPressed() const { return left_.isPressed(); }

void Manager::latchLimitFault() { limit_fault_ = true; }

void Manager::setLimitFault(bool fault) { limit_fault_ = fault; }

bool Manager::getLimitFault() const { return limit_fault_; }

bool Manager::pressedById(LimitId which) const {
  switch (which) {
    case LimitId::TOP:    return top_.isPressed();
    case LimitId::BOTTOM: return bottom_.isPressed();
    case LimitId::LEFT:   return left_.isPressed();
    case LimitId::RIGHT:  return right_.isPressed();
    default:              return false;
  }
}
