

#ifndef MANAGER_H
#define MANAGER_H

struct Command {
  int x;
  int y;
};

class Manager {
 public:
  Manager();
  // Limit fault flag — set from ISR (volatile)
  void setLimitFault(bool v);
  bool isLimitFault();
  void setCommand(int x, int y);
  Command getCommand() const;
  void setFeedRate(int rate);
  int getFeedRate() const;
  void setEvent(int event);
  int getEvent() const;
  void setCurrentPosition(long x, long y);
  long getCurrentX();
  long getCurrentY();

 private:
  Command curr_command;
  int feed_rate;
  int curr_event;
  long current_x_;
  long current_y_;
  // Set by hardware interrupt when a limit switch trips (active-low)
  volatile bool limit_fault_ = false;
};

#endif  // MANAGER_H