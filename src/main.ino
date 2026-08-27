#include <Arduino.h>

#include "Encoder.h"
#include "LimitSwitch.h"
#include "MotorDriver.h"
#include "Pins.h"
#include "fsm_1.h"
#include "manager.h"

// Set up encoder and motor driver objects with the correct pins. The encoder
// ISRs are wired in main() to call the handleEdge() method on each object.
static Encoder encoderL(pins::ENC_L_A, pins::ENC_L_B);
static Encoder encoderR(pins::ENC_R_A, pins::ENC_R_B);

static MotorDriver motorL(pins::M1_DIR, pins::M1_PWM, true);
static MotorDriver motorR(pins::M2_DIR, pins::M2_PWM, true);

LimitSwitch swTop(pins::SW_TOP);
LimitSwitch swBottom(pins::SW_BOTTOM);
LimitSwitch swLeft(pins::SW_LEFT);
LimitSwitch swRight(pins::SW_RIGHT);

static Manager manager(swTop, swBottom, swLeft, swRight);
// Set up the G1 motion object with the motor and encoder objects.
static G1 g1(motorL, motorR, encoderL, encoderR, manager);

// Set up the G28 motion object with the motor and encoder objects.
static G28 g28(motorL, motorR, encoderL, encoderR, manager);

FSM fsm;

// =====================================================================
//  Limit-switch fault path
// ---------------------------------------------------------------------
//  Three layers, each with one job:
//
//  1. ISR (here). One per switch, on cfg::SW_PRESS_EDGE. Sets a bit in
//     limitEdgeMask and nothing else. On the 9 V motor supply, PWM noise
//     couples into the harness and fires these spuriously, so an ISR is
//     only a HINT that a press may have happened — it can never latch a
//     fault by itself.
//
//  2. Debouncer (LimitSwitch, polled via manager.updateLimits() every
//     loop). A reading must hold for cfg::DEBOUNCE_MS to become the
//     committed state. Microsecond noise glitches never survive it.
//
//  3. Confirmation (loop below). An ISR bit opens a per-switch window of
//     cfg::LIMIT_CONFIRM_MS. If the DEBOUNCED state of that switch reads
//     pressed inside the window, the fault latches. If the window expires
//     unconfirmed, the edge was noise and is counted, not acted on.
//
//  The left and bottom switches during G28 are exempt from faulting 
//  (homing presses switches on purpose), and
//  the debounced states it homes with come from the same layer 2.
//
//  Fault recovery is owned by the FSM: while faulted, the G-code parser
//  keeps running and M999 clears the fault (see FSM::doFault).
// =====================================================================

// One bit per switch; bit index matches LimitId (0=TOP..3=RIGHT).
volatile uint8_t limitEdgeMask = 0;

void isrLimitTop() { limitEdgeMask |= (1 << 0); }
void isrLimitBottom() { limitEdgeMask |= (1 << 1); }
void isrLimitLeft() { limitEdgeMask |= (1 << 2); }
void isrLimitRight() { limitEdgeMask |= (1 << 3); }

// Confirmation-window state, owned by loop().
static uint8_t limitWindowMask = 0;      // which switches are being confirmed
static uint32_t limitWindowStartMs[4];   // when each window opened
static uint16_t limitGlitchCount = 0;    // ISR edges rejected as noise

// Interrupt Service Routines (ISRs) for the encoders. These are called when the
// encoder signals change state, and they call the handleEdge() method on the
// corresponding encoder object to update
void isrEncoderL() { encoderL.handleEdge(); }
void isrEncoderR() { encoderR.handleEdge(); }

void setup()
{
  motorL.begin();
  motorR.begin();
  Serial.begin(cfg::SERIAL_BAUD);

  noInterrupts(); // load-bearing, see below

  manager.beginLimits(); // pullups on
  encoderL.begin();
  encoderR.begin();

  delayMicroseconds(500); // let the pullups pull the lines high

  attachInterrupt(digitalPinToInterrupt(pins::ENC_L_A), isrEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pins::ENC_R_A), isrEncoderR, CHANGE);

  // Press edge derives from cfg::SW_PRESSED_LEVEL: the switches are
  // normally-closed to GND, so the line RISES when one is pressed.
  attachInterrupt(digitalPinToInterrupt(pins::SW_TOP), isrLimitTop,
                  cfg::SW_PRESS_EDGE);
  attachInterrupt(digitalPinToInterrupt(pins::SW_BOTTOM), isrLimitBottom,
                  cfg::SW_PRESS_EDGE);
  attachInterrupt(digitalPinToInterrupt(pins::SW_LEFT), isrLimitLeft,
                  cfg::SW_PRESS_EDGE);
  attachInterrupt(digitalPinToInterrupt(pins::SW_RIGHT), isrLimitRight,
                  cfg::SW_PRESS_EDGE);

  EIFR = 0xFF;                  // write-1-to-clear every pending INT0..INT7
  limitEdgeMask = 0;            // discard anything that slipped through
  manager.setLimitFault(false);

  interrupts();

  fsm.setMotion(g1);
  fsm.setMotion2(g28);
  fsm.setManager(manager);
}

void loop()
{
  const uint32_t nowMs = millis();

  // Layer 2: run the debouncers every pass, in every state, so the
  // confirmation below always has a fresh debounced state to consult.
  manager.updateLimits(nowMs);

  // Layer 1 -> 3 handoff: atomically collect any ISR edges.
  uint8_t edges;
  noInterrupts();
  edges = limitEdgeMask;
  limitEdgeMask = 0;
  interrupts();

  // Layer 3: confirm or reject each hinted switch independently.
  for (uint8_t i = 0; i < 4; ++i) {
    const uint8_t bit = (1 << i);

    // A fresh ISR edge opens this switch's confirmation window.
    if ((edges & bit) && !(limitWindowMask & bit)) {
      limitWindowMask |= bit;
      limitWindowStartMs[i] = nowMs;
    }

    if (!(limitWindowMask & bit)) {
      continue;
    }

    const LimitId id = static_cast<LimitId>(i);

    if (manager.pressedById(id)) {
      // The debouncer agrees: this is a real press, not motor noise.
      limitWindowMask &= ~bit;

      if (fsm.getState() == State::G28){
        // During homing, only the switch expected by the
        // current G28 phase is allowed.
        if (!g28.isExpectedLimit(id)) {
            manager.latchLimitFault();
        }
      }
      else {  // Outside G28, every limit press is a fault.
        manager.latchLimitFault();
      }
    } else if ((nowMs - limitWindowStartMs[i]) >= cfg::LIMIT_CONFIRM_MS) {
      // Window expired with no debounced press: the edge was noise.
      limitWindowMask &= ~bit;
      ++limitGlitchCount;
    }
  }

  if (manager.getLimitFault()) {
    fsm.handleEvent(-1);
  }

  fsm.dispatch();

  // The FSM clears the fault on M999 (see FSM::doFault). When it leaves
  // FAULT, drop any half-open confirmation windows and pending ISR edges
  // so a switch still held down can't immediately re-latch the fault.
  static State prevState = State::HOLD;
  const State nowState = fsm.getState();
  if (prevState == State::FAULT && nowState != State::FAULT) {
    limitWindowMask = 0;
    noInterrupts();
    limitEdgeMask = 0;
    interrupts();
  }
  prevState = nowState;
}
