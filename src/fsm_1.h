#pragma once
#include "manager.h"

class G1;
class G28;  // <-- add forward declaration
struct GCodeCommand;

enum class State { IDLE, G1, G28, FAULT };

class FSM {
 public:
  FSM();
  void setMotion(G1& g1);
  void setMotion2(G28& g28);  // <-- add
  void setManager(Manager& manager);
  void handleEvent(int event);
  void dispatch();
  State getState() const;

 private:
  void doIdle();
  void doG1();
  void doG28();
  void doFault();

  State state = State::IDLE;
  G1* g1_ = nullptr;
  G28* g28_ = nullptr;  // <-- add
  Manager* manager_ = nullptr;
  GCodeCommand* command_ = nullptr;
};