#ifndef MANAGER_H
#define MANAGER_H

#include <Arduino.h>

#include "LimitSwitch.h"

struct Command {
  int x;
  int y;
  int feed_rate;
};

// One physical limit switch. The order matters: it lines up with the ISR
// bit positions in main.ino (bit 0 is TOP through to bit 3, RIGHT).
enum class LimitId : uint8_t {
  TOP = 0,
  BOTTOM = 1,
  LEFT = 2,
  RIGHT = 3
};

class Manager {
 public:
  Manager(LimitSwitch& top, LimitSwitch& bottom, LimitSwitch& left,
          LimitSwitch& right);

  // Position management
  void setCurrentPosition(long x, long y);
  long getCurrentX() const;
  long getCurrentY() const;
  void resetXY();

  // Command management
  void setCommand(int x, int y, int feed_rate);
  Command getCommand() const;

  // Limit switch management
  void beginLimits();
  void updateLimits(uint32_t nowMs);

  bool bottomPressed() const;
  bool leftPressed() const;

  // Set by the confirmation window in main.ino. M999 clears it again
  // through setLimitFault(false).
  void latchLimitFault();
  void setLimitFault(bool fault);
  bool getLimitFault() const;

  int getMaxX() const;
  int getMaxY() const;

  // Debounced state of one switch. main.ino's fault check goes through
  // here so it sees the same state G28 homes against.
  bool pressedById(LimitId which) const;

 private:
  bool limit_fault_ = false;

  long current_x_ = 0;
  long current_y_ = 0;

  Command curr_command;

  LimitSwitch& top_;
  LimitSwitch& bottom_;
  LimitSwitch& left_;
  LimitSwitch& right_;
  int max_x = 210;  // in mm 
  int max_y = 135;  // in mm
};

#endif  // MANAGER_H
