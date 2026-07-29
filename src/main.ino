#include <Arduino.h>

#include "Encoder.h"
#include "G1.h"
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

// Set up the FSM and Manager objects.

// Interrupt Service Routines (ISRs) for the encoders. These are called when the
// encoder signals change state, and they call the handleEdge() method on the
// corresponding encoder object to update
void isrEncoderL() { encoderL.handleEdge(); }
void isrEncoderR() { encoderR.handleEdge(); }

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
  motorL.begin();
  motorR.begin();

  fsm.setMotion(g1);
  fsm.setManager(
      manager);
}

void loop() {
  fsm.dispatch();  // HOLD blocks on serial internally until a valid command
                   // arrives
}