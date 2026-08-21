#include <Arduino.h>

#include "Encoder.h"
#include "Kinematics.h"
#include "LimitSwitch.h"
#include "MotorDriver.h"
#include "PID.h"
#include "Pins.h"
#include "Timer.h"
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

volatile bool limitFaultPending = false;

void isrLimitTop() { limitFaultPending = true; }
void isrLimitBottom() { limitFaultPending = true; }
void isrLimitLeft() { limitFaultPending = true; }
void isrLimitRight() { limitFaultPending = true; }

// Set up the FSM and Manager objects.

// Interrupt Service Routines (ISRs) for the encoders. These are called when the
// encoder signals change state, and they call the handleEdge() method on the
// corresponding encoder object to update
void isrEncoderL() { encoderL.handleEdge(); }
void isrEncoderR() { encoderR.handleEdge(); }

const uint32_t PRINT_INTERVAL_MS = 200;
uint32_t lastPrintMs = 0;

void setup() {
  Serial.begin(cfg::SERIAL_BAUD);
  Serial.println(F("BOOT"));
  while (Serial.available() == 0) {
  }  // wait for any input
  while (Serial.available() > 0) Serial.read();  // clear buffer

  manager.beginLimits();
  encoderL.begin();
  encoderR.begin();
  attachInterrupt(digitalPinToInterrupt(pins::ENC_L_A), isrEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pins::ENC_R_A), isrEncoderR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pins::SW_TOP), isrLimitTop, FALLING);
  attachInterrupt(digitalPinToInterrupt(pins::SW_BOTTOM), isrLimitBottom,
                  FALLING);
  attachInterrupt(digitalPinToInterrupt(pins::SW_LEFT), isrLimitLeft, FALLING);
  attachInterrupt(digitalPinToInterrupt(pins::SW_RIGHT), isrLimitRight,
                  FALLING);
  motorL.begin();
  motorR.begin();

  fsm.setMotion(g1);
  fsm.setMotion2(g28);
  fsm.setManager(manager);
}

void loop() {
  while (Serial.available() == 0) {
    // wait for a serial command
  }

  // Handle the event and dispatch the current state
  if (limitFaultPending) {
    noInterrupts();
    limitFaultPending = false;
    interrupts();
    manager.setLimitFault(true);
  }

  if (manager.getLimitFault() && fsm.getState() != State::G28) {
    fsm.handleEvent(-1);
  } else if (fsm.getState() != State::FAULT) {
    fsm.handleEvent(1);
  }
  fsm.dispatch();
}