#include "fsm_1.h"

#include <Arduino.h>

// =====================================================================
// FSM
// =====================================================================

FSM::FSM() = default;

// =====================================================================
// Dependency setup
// =====================================================================

void FSM::setMotion(G1& g1) { g1_ = &g1; }

void FSM::setMotion2(G28& g28) { g28_ = &g28; }

void FSM::setManager(Manager& manager) { manager_ = &manager; }

// =====================================================================
// Event handling
// =====================================================================

void FSM::handleEvent(int event) {
  if (event == -1) {
    if (state != State::FAULT) {          // only on entry
      Serial.print(F("FAULT: limit hit ("));
      if (manager_ != nullptr) {
        Serial.print(manager_->faultSwitchName());
      }
      Serial.println(F(") — send 'r' to recover"));
    }
    state = State::FAULT;
    return;
  }

  switch (state) {
    case State::HOLD: {
      if (event == 1)
        state = State::G1;
      else if (event == 2)
        state = State::G28;
      else if (event == 3)
        state = State::MANUAL;

      break;
    }

    case State::G1: {
      if (event == 0) state = State::HOLD;

      break;
    }

    case State::G28: {
      if (event == 0) state = State::HOLD;

      break;
    }

    case State::FAULT: {
      if (event == 0) state = State::HOLD;

      break;
    }

    case State::MANUAL: {
      if (event == 0) state = State::HOLD;

      break;
    }
  }
}

// =====================================================================
// State dispatch
// =====================================================================

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
      doFault();  // entry already announced; no per-loop spam
      break;

    case State::MANUAL:
      doManual();
      break;
  }
}

// =====================================================================
// State getter
// =====================================================================

State FSM::getState() const { return state; }

// =====================================================================
// State handlers
// =====================================================================

void FSM::doHold() {
  Serial.println(F("in HOLD"));

  // Call parser to check for new commands.
  // Use Manager to read the G-code command and update the FSM event.
}

void FSM::doG1() {
  if (g1_ == nullptr) {
    return;
  }

  g1_->execute(50, 50);

  if (g1_->isComplete()) {
    g1_->reset();
    handleEvent(0);
  }
}

void FSM::doG28() {
  Serial.println(F("in G28"));

  if (g28_ == nullptr) return;

  // Run one non-blocking homing update.
  g28_->execute();

  if (g28_->isComplete()) {
    g28_->reset();
    handleEvent(0);
  }
}

void FSM::doFault() {
  if (g1_ != nullptr)  g1_->reset();
  if (g28_ != nullptr) g28_->reset();
}

void FSM::doManual() { Serial.println(F("in MANUAL")); }