#include "fsm_1.h"

#include <Arduino.h>

// FSM implementation
FSM::FSM(Manager& manager) : manager_(manager) {}

// Set the G1 motion object for the FSM
void FSM::setMotion(G1& g1) { g1_ = &g1; }

// Handle events and transition between states
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

// Dispatch the current state to the appropriate handler
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
      break;
  }
}

// Get the current state of the FSM
State FSM::getState() const { return state; }

// State handler implementations
void FSM::doHold() {
  Serial.println(F("in HOLD"));
  if (manager_.isLimitFault()) {
    Serial.println(F("Limit fault detected!"));
    state = State::FAULT;  // Force transition to FAULT state
    return;
  }
}

void FSM::doG1() {
  Serial.println(F("in G1"));
  if (g1_ != nullptr) {
    g1_->execute(50, 50);
  }
}

void FSM::doG28() { Serial.println(F("in G28")); }

void FSM::doFault() { Serial.println(F("in FAULT")); }

void FSM::doManual() { Serial.println(F("in MANUAL")); }
