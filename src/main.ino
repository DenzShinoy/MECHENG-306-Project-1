#include <Arduino.h>

#include "Encoder.h"
#include "Kinematics.h"
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

FSM fsm;
Manager manager;

// Set up the G1 motion object with the motor and encoder objects.
static G1 g1(motorL, motorR, encoderL, encoderR, manager);

int test_event = 0;  // Event to test the FSM transitions. Change this value to
                     // simulate different events in the loop() function.

// Interrupt Service Routines (ISRs) for the encoders. These are called when the
// encoder signals change state, and they call the handleEdge() method on the
// corresponding encoder object to update
void isrEncoderL() { encoderL.handleEdge(); }
void isrEncoderR() { encoderR.handleEdge(); }

// Limit switch ISRs: set the manager's fault flag (switches are active LOW)
void isrLimitTop() { manager.setLimitFault(true); }
void isrLimitBottom() { manager.setLimitFault(true); }
void isrLimitLeft() { manager.setLimitFault(true); }
void isrLimitRight() { manager.setLimitFault(true); }

void setup() {
  Serial.begin(cfg::SERIAL_BAUD);
  Serial.println(F("BOOT"));
  while (Serial.available() == 0) {
  }  // wait for any input
  while (Serial.available() > 0) Serial.read();  // clear buffer

  encoderL.begin();
  encoderR.begin();
  attachInterrupt(digitalPinToInterrupt(pins::ENC_L_A), isrEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pins::ENC_R_A), isrEncoderR, CHANGE);
  // Configure limit switch pins and attach interrupts (active LOW)
  pinMode(pins::SW_TOP, INPUT_PULLUP);
  pinMode(pins::SW_BOTTOM, INPUT_PULLUP);
  pinMode(pins::SW_LEFT, INPUT_PULLUP);
  pinMode(pins::SW_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pins::SW_TOP), isrLimitTop, FALLING);
  attachInterrupt(digitalPinToInterrupt(pins::SW_BOTTOM), isrLimitBottom,
                  FALLING);
  attachInterrupt(digitalPinToInterrupt(pins::SW_LEFT), isrLimitLeft, FALLING);
  attachInterrupt(digitalPinToInterrupt(pins::SW_RIGHT), isrLimitRight,
                  FALLING);
  motorL.begin();
  motorR.begin();

  fsm.setMotion(g1);
}

void loop() {
  while (Serial.available() == 0) {
    // wait for a serial command
  }
  if (manager.isLimitFault()) {
    Serial.println(F("FAULT: Limit switch triggered!"));
    // Clear the limit fault flag for testing purposes
    test_event = -1;  // Force the FSM into the FAULT state
  }
  // Handle the event and dispatch the current state
  fsm.handleEvent(test_event);
  fsm.dispatch();
}