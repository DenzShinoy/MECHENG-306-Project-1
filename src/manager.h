

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
  long getCurrentX();
  long getCurrentY();

 private:
  Command curr_command;
  int curr_event;
};

#endif  // MANAGER_H