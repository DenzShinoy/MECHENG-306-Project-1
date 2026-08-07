#include "fsm_1.h"

#include <iostream>

void FSM::handleEvent(int event) {
  // event == -1 always forces FAULT (from any state)
  if (event == -1) {
    state = State::FAULT;
    return;
  }

  switch (state) {
    case State::HOLD:
      if (event == 1)
        state = State::G1;
      else if (event == 2)
        state = State::G28;
      else if (event == 3)
        state = State::MANUAL;
      break;
    case State::G1:
      if (event == 0) state = State::HOLD;
      break;
    case State::G28:
      if (event == 0) state = State::HOLD;
      break;
    case State::FAULT:
      if (event == 0) state = State::HOLD;
      break;
    case State::MANUAL:
      if (event == 0) state = State::HOLD;
      break;
  }
}

void FSM::dispatch() {
  switch (state) {
    case State::HOLD:
      doHold();
      break;
    case State::G1:
      doG1();
      break;
    case State::G28:
      doG28();
      break;
    case State::FAULT:
      doFault();
      break;
    case State::MANUAL:
      doManual();
  }
}

State FSM::getState() const { return state; }

void FSM::doHold() { std::cout << "in HOLD\n"; }
void FSM::doG1() { std::cout << "in G1\n"; }
void FSM::doG28() { std::cout << "in G28\n"; }
void FSM::doFault() { std::cout << "in FAULT\n"; }
void FSM::doManual() { std::cout << "in MANUAL\n"; }