#include <Arduino.h>

#include "Encoder.h"
#include "LimitSwitch.h"
#include "MotorDriver.h"
#include "Pins.h"
#include "fsm_1.h"
#include "manager.h"

// Encoders and motors. The encoder ISRs further down just forward to
// handleEdge() on the matching object.
static Encoder encoderL(pins::ENC_L_A, pins::ENC_L_B);
static Encoder encoderR(pins::ENC_R_A, pins::ENC_R_B);

static MotorDriver motorL(pins::M1_DIR, pins::M1_PWM, true);
static MotorDriver motorR(pins::M2_DIR, pins::M2_PWM, true);

LimitSwitch swTop(pins::SW_TOP);
LimitSwitch swBottom(pins::SW_BOTTOM);
LimitSwitch swLeft(pins::SW_LEFT);
LimitSwitch swRight(pins::SW_RIGHT);

static Manager manager(swTop, swBottom, swLeft, swRight);

// The two motion handlers. Both share the same motors and encoders.
static G1 g1(motorL, motorR, encoderL, encoderR, manager);
static G28 g28(motorL, motorR, encoderL, encoderR, manager);

FSM fsm;

// The limit-switch fault path, which ended up in three layers.
//
// The ISRs below are the first one. One per switch, on the press edge,
// and all they do is set a bit. On the 9 V supply the motor PWM couples
// into the switch loom and fires these off when nothing has been touched,
// so an ISR is only ever a hint that something might have happened. On
// its own it can't latch a fault.
//
// Second is the debouncer in LimitSwitch, polled from
// manager.updateLimits() every loop. A reading has to hold for
// cfg::DEBOUNCE_MS before it counts, which glitches never manage.
//
// Third is the confirmation down in loop(). An ISR bit opens a window of
// cfg::LIMIT_CONFIRM_MS for that switch. If the debounced state reads
// pressed inside the window, the fault latches. If the window runs out
// first, it was noise, so we count it and carry on.
//
// The left and bottom switches are exempt during G28, since homing
// presses them on purpose, and homing reads the same debounced state
// anyway.
//
// Getting out of a fault is the FSM's job: the parser keeps running while
// faulted, and M999 clears it (FSM::doFault).

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

// Encoder ISRs. Every edge on an A channel lands in one of these.
void isrEncoderL() { encoderL.handleEdge(); }
void isrEncoderR() { encoderR.handleEdge(); }

void setup()
{
  motorL.begin();
  motorR.begin();
  Serial.begin(cfg::SERIAL_BAUD);

  noInterrupts();  // this matters, see the EIFR clear below

  manager.beginLimits(); // pullups on
  encoderL.begin();
  encoderR.begin();

  delayMicroseconds(500); // let the pullups pull the lines high

  attachInterrupt(digitalPinToInterrupt(pins::ENC_L_A), isrEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pins::ENC_R_A), isrEncoderR, CHANGE);

  // The edge comes from cfg::SW_PRESSED_LEVEL rather than being written
  // out here. Normally-closed to GND, so a press takes the line up.
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

  // Debouncers run every pass in every state, so the check below always
  // has something current to look at.
  manager.updateLimits(nowMs);

  // Grab whatever the ISRs have set since last time.
  uint8_t edges;
  noInterrupts();
  edges = limitEdgeMask;
  limitEdgeMask = 0;
  interrupts();

  // Confirm or throw out each switch on its own.
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
        // While homing, only the switch this phase expects is allowed.
        if (!g28.isExpectedLimit(id)) {
            manager.latchLimitFault();
        }
      }
      else {  // Any other time, a press is a fault, full stop.
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

  // M999 clears the fault over in the FSM. On the way out of FAULT, bin
  // any half-open windows and pending edges, or a switch that's still
  // held down just trips it again straight away.
  static State prevState = State::HOLD;
  const State nowState = fsm.getState();
  if (prevState == State::FAULT && nowState != State::FAULT) {
    Serial.println(F("MUST HOME: G28 before any G1"));
    limitWindowMask = 0;
    noInterrupts();
    limitEdgeMask = 0;
    interrupts();
  }
  prevState = nowState;
}
