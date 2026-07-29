#pragma once
#include <Arduino.h>

// =====================================================================
//  Module 9 — StateMachine
// ---------------------------------------------------------------------
//  Explicit FSM for the plotter, our own code. Holds only the current
//  state and applies transitions in response to events dispatched by the
//  controller. No hardware access here — it is pure logic, which keeps
//  the transition table testable in isolation.
//
//     States : IDLE, HOMING, MOVING, FAULT
//     Events : HOME_CMD, MOVE_CMD, TARGET_REACHED, HOMED,
//              LIMIT_HIT, FAULT_CLEARED
//
//  Transition sketch:
//     IDLE   --HOME_CMD-->  HOMING
//     IDLE   --MOVE_CMD-->  MOVING
//     HOMING --HOMED-->     IDLE
//     MOVING --TARGET_REACHED--> IDLE
//     any    --LIMIT_HIT (unexpected)--> FAULT
//     FAULT  --FAULT_CLEARED--> IDLE
// =====================================================================

class StateMachine {
public:
  enum class State : uint8_t { IDLE, HOMING, MOVING, FAULT };
  enum class Event : uint8_t {
    HOME_CMD, MOVE_CMD, TARGET_REACHED, HOMED, LIMIT_HIT, FAULT_CLEARED
  };

  StateMachine();

  // Apply one event to the transition table. Returns the resulting state.
  // Ignores events that are invalid for the current state.
  State dispatch(Event e);

  State current() const;

private:
  State _state;
};
