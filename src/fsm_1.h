#ifndef FSM_H
#define FSM_H

#include "G1.h"

enum class State { HOLD, G1, G28, FAULT, MANUAL };

class FSM {
 public:
  FSM(Manager& manager);        // Constructor
  void handleEvent(int event);  // Handle events and transition between states
  void dispatch();  // Dispatch the current state to the appropriate handler
  State getState() const;  // Get the current state of the FSM
  void setMotion(G1& g1);  // Set the G1 motion object for the FSM

 private:
  void doHold();
  void doG1();
  void doG28();
  void doFault();
  void doManual();

  State state = State::HOLD;
  G1* g1_ = nullptr;
  Manager& manager_;
};

#endif  // FSM_H