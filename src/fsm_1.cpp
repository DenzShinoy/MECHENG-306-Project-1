#include "fsm_1.h"

#include <Arduino.h>

<<<<<<< HEAD
#include "G1.h"
#include "GCodeParser.h"

FSM::FSM() : command_(new GCodeCommand()) {}
=======
/* To do :
Implement Gcode parser into manager class and then use manager class to update
FSM state (check if this needs to go into this) based on Gcode command received.

Use position tracker to update current position of the machine after each move
command (G1) and after homing (G28)

Report stuff

*/
// FSM implementation
FSM::FSM() = default;
>>>>>>> 3d2ac2e (updated some stuff)

void FSM::setMotion(G1& g1) { g1_ = &g1; }
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

<<<<<<< HEAD
void FSM::doIdle() {
  Serial.println(F("in IDLE"));
  if (manager_ == nullptr) return;

  int event = GcodeParserFull(*command_, *manager_);
  if (event != kNoEvent) {
    handleEvent(event);
  }
}

void FSM::doG1() {
  Serial.println(F("in G1"));
  if (g1_ == nullptr || manager_ == nullptr) {
    state = State::FAULT;
    return;
  }

  Command target = manager_->getCommand();
  g1_->execute(target.x, target.y);

  handleEvent(0);
}

void FSM::doG28() { Serial.println(F("in G28")); }

void FSM::doFault() { Serial.println(F("in FAULT")); }
void FSM::doManual() { Serial.println(F("in MANUAL")); }
=======
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

      // use manager class to read instance of GCodeCommand and update FSM state
      // accordingly}
    }
    void FSM::doG1() {
      Serial.println(F("in G1"));
      if (g1_ != nullptr) {
        // Give intial encoder counts to pos tracking function

        // Call G1 function
        g1_->execute(50, 50);

        // Give final encoder counts to pos tracking function
      }
    }
    void FSM::doG28() {  // Placeholder for G28 state logic
      Serial.println(F("in G28"));
    }
    void FSM::doFault() {  // Placeholder for FAULT state logic
      Serial.println(F("in FAULT"));
    }
    void FSM::doManual() {  // Placeholder for MANUAL state logic
      Serial.println(F("in MANUAL"));
    }
    void FSM::doG28() { Serial.println(F("in G28")); }
    void FSM::doFault() { Serial.println(F("in FAULT")); }
    void FSM::doManual() { Serial.println(F("in MANUAL")); }
>>>>>>> 3d2ac2e (updated some stuff)
