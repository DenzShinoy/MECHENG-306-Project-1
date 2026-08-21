#include "fsm_1.h"

#include <Arduino.h>

#include "G1.h"
#include "G28.h"
#include "GCodeParser.h"

FSM::FSM() : command_(new GCodeCommand()) {}

void FSM::setMotion(G1& g1) { g1_ = &g1; }
void FSM::setMotion2(G28& g28) { g28_ = &g28; }
void FSM::setManager(Manager& manager) { manager_ = &manager; }

void FSM::handleEvent(int event) {
  if (event == -1) {
    state = State::FAULT;
    return;
  }

  switch (state) {
    case State::IDLE:
      if (event == 1)
        state = State::G1;
      else if (event == 2)
        state = State::G28;
      else if (event == 3)
        state = State::MANUAL;
      break;
    case State::G1:
      if (event == 0) state = State::IDLE;
      break;
    case State::G28:
      if (event == 0) state = State::IDLE;
      break;
    case State::FAULT:
      if (event == 0) state = State::IDLE;
      break;
    case State::MANUAL:
      if (event == 0) state = State::IDLE;
      break;
  }
}

void FSM::dispatch() {
  // Always drain + validate serial, even mid-move, so the 64-byte
  // AVR hardware RX buffer never silently overflows.
  if (manager_ != nullptr && state != State::IDLE) {
    int event = GcodeParserFull(*command_, *manager_);
    if (event != kNoEvent) {
      Serial.println(F("Error: machine busy, command ignored."));
    }
  }

  switch (state) {
    case State::IDLE:
      doIdle();
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

State FSM::getState() const { return state; }

void FSM::doIdle() {
  if (manager_ == nullptr) return;

  int event = GcodeParserFull(*command_, *manager_);
  if (event != kNoEvent) {
    handleEvent(event);
  }
}

void FSM::doG1() {
  if (g1_ == nullptr || manager_ == nullptr) {
    state = State::FAULT;
    return;
  }

  static bool wasActive = false;
  if (!wasActive) {
    g1_->reset();
  }
  wasActive = true;

  Command target = manager_->getCommand();
  g1_->execute(target.x, target.y, target.feed_rate);

  if (g1_->isComplete()) {
    wasActive = false;
    handleEvent(0);
  }
}


void FSM::doG28() {
  if (g28_ == nullptr) {
    state = State::FAULT;
    return;
  }
  g28_->execute();

  if (g28_->isComplete()) {
    handleEvent(0);
  }
}

void FSM::doFault() { Serial.println(F("in FAULT")); }
void FSM::doManual() { Serial.println(F("in MANUAL")); }