#ifndef FSM_H
#define FSM_H

enum class State { HOLD, G1, G28, FAULT, MANUAL };

class FSM {
 public:
  FSM();
  void handleEvent(int event);
  void dispatch();
  State getState() const;

 private:
  void doHold();
  void doG1();
  void doG28();
  void doFault();
  void doManual();

  State state = State::HOLD;
};

#endif  // FSM_H