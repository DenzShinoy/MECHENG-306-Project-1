// =====================================================================
//  Module 11 — main sketch  (setup / loop)
// ---------------------------------------------------------------------
//  Composition root. This is the ONLY place that constructs concrete
//  objects and knows real pin numbers; it injects them into the
//  PlotterController and then does nothing but forward time each loop.
//  ISRs live here as free functions (attachInterrupt cannot take a
//  member) and simply forward to the right Encoder instance.
//
//  No control logic here — setup() wires, loop() ticks.
// =====================================================================

#include <Arduino.h>
#include "Pins.h"
#include "Encoder.h"
#include "LimitSwitch.h"
#include "MotorDriver.h"
#include "PIDController.h"
#include "TrajectoryPlanner.h"
#include "GCodeParser.h"
#include "StateMachine.h"
#include "PlotterController.h"

// --- Concrete modules (static storage, no dynamic allocation) --------
static Encoder     encL(pins::ENC_L_A, pins::ENC_L_B);
static Encoder     encR(pins::ENC_R_A, pins::ENC_R_B);

static MotorDriver motL(pins::M1_DIR, pins::M1_PWM);
static MotorDriver motR(pins::M2_DIR, pins::M2_PWM);

static PIDController pidL(cfg::PID_KP, cfg::PID_KI, cfg::PID_KD,
                         -cfg::PWM_LIMIT, cfg::PWM_LIMIT);
static PIDController pidR(cfg::PID_KP, cfg::PID_KI, cfg::PID_KD,
                         -cfg::PWM_LIMIT, cfg::PWM_LIMIT);

static LimitSwitch swTop(pins::SW_TOP);
static LimitSwitch swBottom(pins::SW_BOTTOM);
static LimitSwitch swLeft(pins::SW_LEFT);
static LimitSwitch swRight(pins::SW_RIGHT);

static TrajectoryPlanner planner(cfg::MAX_VEL_CPS, cfg::MAX_ACC_CPS2);
static GCodeParser       parser;
static StateMachine      fsm;

static PlotterController controller(
    encL, encR, motL, motR, pidL, pidR,
    swTop, swBottom, swLeft, swRight,
    planner, parser, fsm);

// --- ISR trampolines: forward the A-channel edge to the encoder ------
static void isrEncLeft()  { encL.handleEdge(); }
static void isrEncRight() { encR.handleEdge(); }

void setup() {
  controller.begin();
  // TODO: attachInterrupt(digitalPinToInterrupt(pins::ENC_L_A),
  //                       isrEncLeft,  CHANGE);   // 2x decode
  // TODO: attachInterrupt(digitalPinToInterrupt(pins::ENC_R_A),
  //                       isrEncRight, CHANGE);
}

void loop() {
  controller.update(millis());
}
