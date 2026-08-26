#pragma once
#include "G1.h"
#include "G28.h"
#include "manager.h"

// class G1;
// class G28;  // <-- add forward declaration
struct GCodeCommand;

enum class State { HOLD, G1, G28, FAULT };

class FSM {
 public:
  FSM();                        // Constructor
  void handleEvent(int event);  // Handle events and transition between states
  void dispatch();  // Dispatch the current state to the appropriate handler
  State getState() const;             // Get the current state of the FSM
  void setMotion(G1& g1);             // Set the G1 motion object for the FSM
  void setMotion2(G28& g28);          // Set the G1 motion object for the FSM
  void setManager(Manager& manager);  // Set the Manager object for the FSM

 private:
  void doHold();
  void doG1();
  void doG28();
  void doFault();

  State state = State::HOLD;
  G1* g1_ = nullptr;
  G28* g28_ = nullptr;
  Manager* manager_ = nullptr;
  GCodeCommand* command_ = nullptr;
};
