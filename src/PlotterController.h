#pragma once
#include <Arduino.h>

#include "Encoder.h"
#include "GCodeParser.h"
#include "Kinematics.h"
#include "LimitSwitch.h"
#include "MotorDriver.h"
#include "PIDController.h"
#include "StateMachine.h"
#include "TrajectoryPlanner.h"

// =====================================================================
//  Module 10 — PlotterController
// ---------------------------------------------------------------------
//  Top-level orchestrator. Owns no hardware detail of its own; instead
//  the concrete modules are constructed in main and injected here by
//  reference (dependency injection), which keeps this class testable and
//  the wiring visible in one place. Responsibilities:
//    - pump serial -> GCodeParser -> command
//    - drive the StateMachine from commands and sensor events
//    - per control tick: read encoders, advance the trajectory, run the
//      two PIDs, write the motors
//    - run homing against the limit switches
//  Everything is polled and non-blocking; update() must return quickly.
// =====================================================================

class PlotterController {
 public:
  // All collaborators injected by reference. `L`/`R` = left/right motor
  // chains; `top/bottom/left/right` = the four limit switches.
  PlotterController(Encoder& encL, Encoder& encR, MotorDriver& motL,
                    MotorDriver& motR, PIDController& pidL, PIDController& pidR,
                    LimitSwitch& swTop, LimitSwitch& swBottom,
                    LimitSwitch& swLeft, LimitSwitch& swRight,
                    TrajectoryPlanner& planner, StateMachine& fsm);

  // begin() all owned-by-reference modules and set pin/serial state.
  void begin();

  // One pass of the super-loop: called as fast as possible from loop().
  // Internally gates the control math to cfg::CONTROL_PERIOD_MS.
  void update(uint32_t nowMs);

 private:
  // --- internal steps, split for clarity (all non-blocking) ---
  void pumpSerial();  // read a line, parse, enqueue a command
  void handleCommand(const struct GCodeCommand& cmd);
  void serviceHoming(uint32_t nowMs);   // seek switches, set datum
  void runControlTick(uint32_t nowMs);  // trajectory -> PID -> motors
  void updateSwitches(uint32_t nowMs);
  void enterFault();  // stop motors, force FSM to FAULT

  // Injected collaborators.
  Encoder& _encL;
  Encoder& _encR;
  MotorDriver& _motL;
  MotorDriver& _motR;
  PIDController& _pidL;
  PIDController& _pidR;
  LimitSwitch& _swTop;
  LimitSwitch& _swBottom;
  LimitSwitch& _swLeft;
  LimitSwitch& _swRight;
  TrajectoryPlanner& _planner;
  StateMachine& _fsm;

  // Scheduling + serial line buffer (fixed size, no dynamic alloc).
  uint32_t _lastControlMs = 0;
  char _lineBuf[64];
  uint8_t _lineLen = 0;
};
