#ifndef MANAGER_H
#define MANAGER_H

struct Command {
  int x;
  int y;
};

class Manager {
 public:
  Manager();
  void setCommand(int x, int y);
  Command getCommand() const;
  void setFeedRate(int rate);
  int getFeedRate() const;
  void setEvent(int event);
  int getEvent() const;
  void setCurrentPosition(long x, long y);
  long getCurrentX() const;
  long getCurrentY() const;

 private:
  Command curr_command;
  int curr_event;
  long current_x_ = 0;
  long current_y_ = 0;
  int feed_rate_ = 0;
};

#endif  // MANAGER_H