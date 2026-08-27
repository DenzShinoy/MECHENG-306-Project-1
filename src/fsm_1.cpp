#include "fsm_1.h"

#include <Arduino.h>

#include "G1.h"
#include "G28.h"
#include "GCodeParser.h"

FSM::FSM() : command_(new GCodeCommand()) {}

void FSM::setMotion(G1& g1) { g1_ = &g1; }
void FSM::setMotion2(G28& g28) { g28_ = &g28; }

void FSM::setManager(Manager& manager) { manager_ = &manager; }

// =====================================================================
// Event handling
// =====================================================================

void FSM::handleEvent(int event) {
  // event == -1 always forces FAULT from any state.
  if (event == -1) {
    state = State::FAULT;
    return;
  }

  switch (state) {
    case State::HOLD: {
      if (event == 1) {
        state = State::G1;
      } else if (event == 2) {
        state = State::G28;
      }

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
  }
}

// =====================================================================
// State dispatch
// =====================================================================

void FSM::dispatch() {
  reportState();

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
  }
}

State FSM::getState() const { return state; }

// =====================================================================
// Serial reporting
// ---------------------------------------------------------------------
//  The FSM owns the machine's state reporting. It prints once per
//  transition: the state being entered, followed by the one detail that
//  state is responsible for announcing (the G1 target, that G28 is
//  homing, and how to clear a fault).
//
//  The only other thing on the machine that prints is GCodeParser, which
//  reports every line it rejects as a single "ERR: ..." line. The FSM
//  never sees those, since a rejected command produces no state change.
// =====================================================================

void FSM::reportState() {
  if (stateReported_ && reportedState_ == state) {
    return;
  }

  reportedState_ = state;
  stateReported_ = true;

  switch (state) {
    case State::HOLD:
      Serial.println(F("STATE: HOLD"));
      break;

    case State::G1: {
      Serial.println(F("STATE: G1"));

      if (manager_ != nullptr) {
        const Command target = manager_->getCommand();

        Serial.print(F("G1: moving to "));
        Serial.print(target.x);
        Serial.print(F(" "));
        Serial.print(target.y);
        Serial.print(F(" at "));
        Serial.print(target.feed_rate);
        Serial.println(F(" feed rate"));
      }

      break;
    }

    case State::G28:
      Serial.println(F("STATE: G28"));
      Serial.println(F("G28: homing"));
      break;

    case State::FAULT:
      Serial.println(F("STATE: FAULT"));
      Serial.println(F("FAULT: press M999"));
      break;
  }
}

void FSM::doHold() {
  if (manager_ == nullptr) return;

  int event = GcodeParserFull(*command_, *manager_);
  if (event != kNoEvent) {
    handleEvent(event);
  }
}

void FSM::doG28() {
  if (g28_ == nullptr) return;

  // Run one non-blocking homing update.
  g28_->execute();

  if (g28_->isComplete()) {
    g28_->reset();
    handleEvent(0);
  }
}

void FSM::doG1() {
  if (g1_ == nullptr || manager_ == nullptr) {
    state = State::FAULT;
    return;
  }

  Command target = manager_->getCommand();
  g1_->execute(target.x, target.y, target.feed_rate);

  if (g1_->isComplete()) {
    g1_->reset();
    handleEvent(0);
  }
}

void FSM::doFault() {
  // Keep the motors stopped for as long as the fault holds.
  if (g1_ != nullptr) g1_->reset();
  if (g28_ != nullptr) g28_->reset();

  if (manager_ == nullptr) return;

  int event = GcodeParserFull(*command_, *manager_);
  if (event == kNoEvent) return;  // no complete line yet

  if (event == 0) {
    // CLEAR_FAULT (M999) — clear the latched limit fault and return to HOLD.
    manager_->setLimitFault(false);
    handleEvent(event);
  }
  // Any other command is ignored while faulted; the FAULT banner already
  // told the operator to send M999.
}
