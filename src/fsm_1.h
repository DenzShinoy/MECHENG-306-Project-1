#pragma once
#include "manager.h"

class G1;
struct GCodeCommand;  // forward declaration only — full type not needed here

enum class State { HOLD, G1, G28, FAULT, MANUAL };

class FSM {
 public:
  FSM();
  void setMotion(G1& g1);
  void setManager(Manager& manager);
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
  G1* g1_ = nullptr;
  Manager* manager_ = nullptr;
  GCodeCommand* command_ = nullptr;  // can't hold by value with only a forward decl
};