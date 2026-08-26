#ifndef MANAGER_H
#define MANAGER_H

#include <Arduino.h>

#include "LimitSwitch.h"

struct Command {
  int x;
  int y;
  int feed_rate;
};

// Identifies one physical limit switch. Order matches the ISR bit
// positions in main.ino (bit 0 = TOP ... bit 3 = RIGHT).
enum class LimitId : uint8_t { TOP = 0, BOTTOM = 1, LEFT = 2, RIGHT = 3, NONE = 255 };

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
  // Latch a fault caused by one specific switch (remembered for serial
  // reporting until the fault is cleared).
  void latchLimitFault(LimitId which);
  void setLimitFault(bool fault);  // false clears the fault + cause
  bool getLimitFault() const;
  LimitId getFaultSwitch() const;
  const __FlashStringHelper* faultSwitchName() const;

  // Debounced state of one switch by id — used by the fault confirmation
  // in main.ino so it runs off the same debouncer G28 homes with.
  bool pressedById(LimitId which) const;

 private:
  int curr_event;

  bool limit_fault_ = false;
  LimitId fault_switch_ = LimitId::NONE;

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
