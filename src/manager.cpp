#include "manager.h"

Manager::Manager(LimitSwitch& top, LimitSwitch& bottom, LimitSwitch& left,
                 LimitSwitch& right)
    : curr_command{0, 0},
      feed_rate(0),
      curr_event(0),
      top_(top),
      bottom_(bottom),
      left_(left),
      right_(right) {}

void Manager::setCommand(int x, int y) {
  curr_command.x = x;
  curr_command.y = y;
}

Command Manager::getCommand() const { return curr_command; }

void Manager::setFeedRate(int rate) { feed_rate = rate; }

int Manager::getFeedRate() const { return feed_rate; }

// Event management
void Manager::setEvent(int event) { curr_event = event; }

int Manager::getEvent() const { return curr_event; }

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

bool Manager::topPressed() const { return top_.isPressed(); }

bool Manager::bottomPressed() const { return bottom_.isPressed(); }

bool Manager::leftPressed() const { return left_.isPressed(); }

bool Manager::rightPressed() const { return right_.isPressed(); }

bool Manager::topJustPressed() { return top_.justPressed(); }

bool Manager::bottomJustPressed() { return bottom_.justPressed(); }

bool Manager::leftJustPressed() { return left_.justPressed(); }

bool Manager::rightJustPressed() { return right_.justPressed(); }

void Manager::latchLimitFault(LimitId which) {
  limit_fault_ = true;
  fault_switch_ = which;
}

void Manager::setLimitFault(bool fault) {
  limit_fault_ = fault;
  if (!fault) {
    fault_switch_ = LimitId::NONE;
  }
}

bool Manager::getLimitFault() const { return limit_fault_; }

LimitId Manager::getFaultSwitch() const { return fault_switch_; }

const __FlashStringHelper* Manager::faultSwitchName() const {
  switch (fault_switch_) {
    case LimitId::TOP:    return F("TOP (D18)");
    case LimitId::BOTTOM: return F("BOTTOM (D19)");
    case LimitId::LEFT:   return F("LEFT (D20)");
    case LimitId::RIGHT:  return F("RIGHT (D21)");
    default:              return F("none");
  }
}

bool Manager::pressedById(LimitId which) const {
  switch (which) {
    case LimitId::TOP:    return top_.isPressed();
    case LimitId::BOTTOM: return bottom_.isPressed();
    case LimitId::LEFT:   return left_.isPressed();
    case LimitId::RIGHT:  return right_.isPressed();
    default:              return false;
  }
}