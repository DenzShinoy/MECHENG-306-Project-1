#ifndef FSM_H
#define FSM_H

#include "G1.h"

enum class State { HOLD, G1, G28, FAULT, MANUAL };

class FSM {
 public:
  FSM();
  void handleEvent(int event);
  void dispatch();
  State getState() const;
  void setMotion(G1& g1);

 private:
  void doHold();
  void doG1();
  void doG28();
  void doFault();
  void doManual();

  State state = State::HOLD;
  G1* g1_ = nullptr;
};

#endif  // FSM_H