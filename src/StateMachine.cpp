#include "StateMachine.h"

StateMachine::StateMachine() : _state(State::IDLE) {}

StateMachine::State StateMachine::dispatch(Event e) {
  // TODO: switch on (_state, e) and update _state per the table in the
  // header. Any unexpected LIMIT_HIT forces FAULT. Unhandled pairs are
  // no-ops (state unchanged).
  (void)e;
  return _state;
}

StateMachine::State StateMachine::current() const {
  return _state;
}
