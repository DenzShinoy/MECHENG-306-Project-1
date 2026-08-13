#include "fsm_1.h"

#include <Arduino.h>

// FSM implementation
FSM::FSM() = default;

// Set the G1 motion object for the FSM
void FSM::setMotion(G1& g1) { g1_ = &g1; }

void FSM::setMotion2(G28& g28) { g28_ = &g28; }

void FSM::setManager(Manager& manager) { manager_ = &manager; }

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
  }
}

// Get the current state of the FSM
State FSM::getState() const { return state; }

// State handler implementations
void FSM::doHold() {
  Serial.println(F("in HOLD"));
}  // Placeholder for HOLD state logic
void FSM::doG1() {
  Serial.println(F("in G1"));
  if (g1_ != nullptr) {
void FSM::doHold() { 

  Serial.println(F("in HOLD")); 
  // Call parser to check for new commands

  // use manager class to read instance of GCodeCommand and update FSM state accordingly}
}
void FSM::doG1() {
  Serial.println(F("in G1"));
  if (g1_ != nullptr) {
  // Give intial encoder counts to pos tracking function

  // Call G1   function 
    g1_->execute(50, 50);

  // Give final encoder counts to pos tracking function
  }
}
void FSM::doG28() {  // Placeholder for G28 state logic
  Serial.println(F("in G28"));
  if (g28_ != nullptr){
    
    g28_->execute(manager_->getCurrentX(), manager_->getCurrentY()); // can replace with the G1
    // simply add manager_->resetXY(); if replace with G1
  }
}
void FSM::doFault() {  // Placeholder for FAULT state logic
  Serial.println(F("in FAULT"));
}
void FSM::doManual() {  // Placeholder for MANUAL state logic
  Serial.println(F("in MANUAL"));
}
void FSM::doG28() { 
  Serial.println(F("in G28")); }
void FSM::doFault() { Serial.println(F("in FAULT")); }
void FSM::doManual() { Serial.println(F("in MANUAL")); }
