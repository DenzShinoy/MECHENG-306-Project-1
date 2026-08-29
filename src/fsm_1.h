#pragma once
#include "G1.h"
#include "G28.h"
#include "manager.h"

struct GCodeCommand;

enum class State { HOLD, G1, G28, FAULT };

class FSM {
 public:
  FSM();

  // Events come from the parser. -1 is the fault event and always wins.
  void handleEvent(int event);

  // Runs one tick of whatever state we're currently in.
  void dispatch();

  State getState() const;

  void setMotion(G1& g1);
  void setMotion2(G28& g28);
  void setManager(Manager& manager);

 private:
  // Prints the state on the way in, once per transition.
  void reportState();

  void doHold();
  void doG1();
  void doG28();
  void doFault();

  State state = State::HOLD;
  State reportedState_ = State::HOLD;
  bool stateReported_ = false;
  G1* g1_ = nullptr;
  G28* g28_ = nullptr;
  Manager* manager_ = nullptr;
  GCodeCommand* command_ = nullptr;
};
