#ifndef MANAGER_H
#define MANAGER_H

#include <Arduino.h>

#include "LimitSwitch.h"

struct Command {
  int x;
  int y;
  int feed_rate;
};

class Manager {
 public:
  Manager(LimitSwitch& top, LimitSwitch& bottom, LimitSwitch& left,
          LimitSwitch& right);

  // Event management
  void setEvent(int event);
  int getEvent() const;

  // Position management
  void setCurrentPosition(long x, long y);
  long getCurrentX() const;
  long getCurrentY() const;
  void resetXY();

  // Command management
  void setCommand(int x, int y, int feed_rate);
  Command getCommand() const;
  void setFeedRate(int rate);
  int getFeedRate() const;

  // Limit switch management
  void beginLimits();
  void updateLimits(uint32_t nowMs);

  bool topPressed() const;
  bool bottomPressed() const;
  bool leftPressed() const;
  bool rightPressed() const;

  bool topJustPressed();
  bool bottomJustPressed();
  bool leftJustPressed();
  bool rightJustPressed();
  void setLimitFault(bool fault);
  bool getLimitFault() const;

 private:
  int curr_event;

  bool limit_fault_ = false;

  long current_x_ = 0;
  long current_y_ = 0;

  Command curr_command;
  int feed_rate_ = 0;

  LimitSwitch& top_;
  LimitSwitch& bottom_;
  LimitSwitch& left_;
  LimitSwitch& right_;
};

#endif  // MANAGER_H
